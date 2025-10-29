#include <string>
#include "aw_runtime_monitor/planning/planning_shield.hpp"
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_size.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include <chrono> // Required time measurement
#include <spot/kripke/kripkegraph.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/emptiness.hh>

// for CLI interaction with Maude
#include <unistd.h>
#include <pty.h>
#include <sys/wait.h>


// PlanningPoint member methods
PlanningPoint::PlanningPoint(const glm::vec2& _position, float _velocity, float _accel, 
            float _time_to_next_point, float _time_from_start)
    : position(_position), velocity(_velocity), accel(_accel), 
      time_to_next_point(_time_to_next_point), time_from_start(_time_from_start) {
}

// ConstantHeadingVehicle member methods
ConstantHeadingVehicle::ConstantHeadingVehicle(const nlohmann::json& vehicle_current_pose, double veh_length, double veh_width, double veh_center_x, double veh_center_y){
    this->init_pos = glm::vec2(vehicle_current_pose["position"]["x"],
                                vehicle_current_pose["position"]["y"]);
    this->init_rot = vehicle_current_pose["rotation"]["z"];
    this->init_vertices = (getEgoWorldVertices(this->init_pos, this->init_rot,
                            glm::vec2(veh_length, veh_width),
                            glm::vec2(veh_center_x, veh_center_y)));
    double heading_rad = glm::radians(this->init_rot);
    this->norm_vel_ = glm::vec2(std::cos(heading_rad), std::sin(heading_rad));
}

std::vector<glm::vec2> ConstantHeadingVehicle::getVerticesWhenAtPosition(const glm::vec2& position) const{
    glm::vec2 direction = position - this->init_pos;
    std::vector<glm::vec2> result;
    result.reserve(this->init_vertices.size());
    for (const auto& v : this->init_vertices) {
        result.push_back(v + direction);
    }
    return result;
}

std::vector<glm::vec2> ConstantHeadingVehicle::getVerticesAtTime(float time, float speed){
    glm::vec2 velocity = speed * this->norm_vel_;
    std::vector<glm::vec2> result;
    result.reserve(this->init_vertices.size());
    for (const auto& v : this->init_vertices) {
        result.push_back(v + time * velocity);
    }
    return result;
}

/**
 * Helper functions for PlanningTrajectory class
 */
std::tuple<size_t, float> extractClosestPoint(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& raw_points, const glm::vec2& current_pos) {
    float min_dis = 1e9;
    size_t _id = -1;
    for (size_t i = 0; i < raw_points.size(); ++i) {
        const auto& point = raw_points[i];
        glm::vec2 nppoint = extractObjPosition(point);
        float dis = glm::length(current_pos - nppoint);
        if (dis < min_dis) {
            _id = i;
            min_dis = dis;
        }
    }
    return std::make_tuple(_id, min_dis);
}

std::tuple<std::vector<glm::vec2>, std::vector<float>, std::vector<float>> getPosAndVels(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points_raw, size_t start_id=0) {
    std::vector<glm::vec2> positions;
    std::vector<float> long_vels;
    std::vector<float> accels;
    for (size_t i = start_id; i < points_raw.size(); ++i) {
        const auto& point = points_raw[i];
        if (point.lateral_velocity_mps > 0.1) {
            throw std::runtime_error("Not handle case when lateral velocity > 0");
        }
        positions.push_back(glm::vec2(point.pose.position.x, point.pose.position.y));
        long_vels.push_back(point.longitudinal_velocity_mps);
        accels.push_back(point.acceleration_mps2);
    }
    return std::make_tuple(positions, long_vels, accels);
}

float deriveTimeStep(const glm::vec2& position0, float speed0, float accel0, 
                    const glm::vec2& position1, float speed1, float accel1){
    if (accel0 + accel1 == 0.0) {
        if (speed0 + speed1 == 0.0) {
            return 0.0;
        }
        return 2 * float(glm::length(position1 - position0)) / (speed0 + speed1);
    }
    float temp = 2 * (speed1 - speed0) / (accel0 + accel1);
    if (temp < 0) {
        return 2 * float(glm::length(position1 - position0)) / (speed0 + speed1);
    }
    return temp;
}

// PlanningTrajectory member methods
PlanningTrajectory::PlanningTrajectory(const ConstantHeadingVehicle& _ego_current_state) 
    : ego_current_state(_ego_current_state) {
}
PlanningTrajectory::PlanningTrajectory(const ConstantHeadingVehicle& _ego_current_state,
        // const std::vector<PlanningPoint>& _past_points, 
        size_t _start_point_id,
        const std::vector<PlanningPoint>& _future_points) 
    : ego_current_state(_ego_current_state) ,
      start_point_id(_start_point_id),
      future_points(_future_points) {
}

void PlanningTrajectory::parseFuturePoints(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points_raw) {
    auto pva_tuple = getPosAndVels(points_raw);
    std::vector<glm::vec2> positions = std::get<0>(pva_tuple);
    std::vector<float> long_vels = std::get<1>(pva_tuple);
    std::vector<float> accels = std::get<2>(pva_tuple);

    this->future_points.clear();
    float time_from_start = 0.0;
    for (size_t i = 0; i < positions.size(); ++i) {
        float time_step = timestamp(points_raw[i].time_from_start, false);
        if (time_step == 0 && 1 <= i && i <= positions.size() - 2) {
            time_step = deriveTimeStep(
                positions[i], long_vels[i], accels[i], positions[i + 1], long_vels[i + 1], accels[i + 1]);
        } else {
            this->future_points.push_back(PlanningPoint(
                positions[i], long_vels[i], accels[i], time_step, time_from_start
            ));
        }
        time_from_start += time_step;
    }
}

// extract planning trajectory
void PlanningTrajectory::parseFuturePointsWithoutTimesteps(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points_raw, size_t start_id,
        bool skip_tails){
    auto pva_tuple = getPosAndVels(points_raw, start_id);
    std::vector<glm::vec2> positions = std::get<0>(pva_tuple);
    std::vector<float> long_vels = std::get<1>(pva_tuple);
    std::vector<float> accels = std::get<2>(pva_tuple);

    // this->future_points.clear();
    float time_from_start = 0.0;
    size_t bound = positions.size() - 1;
    if (skip_tails) {
        bound -= 2;
    }
    for (size_t i = 0; i < bound; ++i) {
        float time_step = deriveTimeStep(
            positions[i], long_vels[i], accels[i], positions[i + 1], long_vels[i + 1], accels[i + 1]);
        if (time_step < 0) {
            return;
        }
        this->future_points.push_back(PlanningPoint(
            positions[i], long_vels[i], accels[i], time_step, time_from_start
        ));
        time_from_start += time_step;
    }
}

void PlanningTrajectory::parsePastFuturePoints(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points_raw){
    glm::vec2 init_pos = this->ego_current_state.init_pos;

    // find point entry in points_raw that has position closest to current position of Ego
    // because the planning trajectory also includes some previous positions
    auto point_tuple = extractClosestPoint(points_raw, init_pos);
    size_t _id = std::get<0>(point_tuple);

    float current_vel = points_raw[_id].longitudinal_velocity_mps;
    while (current_vel > 0 && _id < points_raw.size() - 1) {
        glm::vec2 pos = extractObjPosition(points_raw[_id]);
        glm::vec2 next_pos = extractObjPosition(points_raw[_id + 1]);
        if (glm::length(pos - next_pos) / current_vel > 0.05) {
            break;
        }
        _id += 1;
    }
    this->start_point_id = _id;
    this->parseFuturePointsWithoutTimesteps(points_raw, _id);
}

// PlanningShield member methods
PlanningTrajectory PlanningShield::toPlanningTrajectoryObj(const autoware_planning_msgs::msg::Trajectory& trajectory_msg, 
        const nlohmann::json& estimated_kinematic, const nlohmann::json& ego_shape){
    float ego_length = ego_shape["size"]["x"];
    float ego_width = ego_shape["size"]["y"];
    float ego_center_x = ego_shape["center"]["x"];
    float ego_center_y = ego_shape["center"]["y"];
    
    ConstantHeadingVehicle ego_current_state = ConstantHeadingVehicle(
        estimated_kinematic["pose"], ego_length, ego_width,
        ego_center_x, ego_center_y);

    PlanningTrajectory planning_points = PlanningTrajectory(ego_current_state);
    planning_points.parsePastFuturePoints(trajectory_msg.points);
    return planning_points;
}


/**
 * Helper functions for PlanningShield class
 */
// Send a Maude command
void sendCommand(int fd, const std::string& cmd) {
    std::string full_cmd = cmd + "\n";
    write(fd, full_cmd.c_str(), full_cmd.size());
}
// Wait until a specific prompt string appears in the Maude output
bool waitForPrompt(int fd, const std::string& prompt, std::string* captured_output = nullptr) {
    std::string buffer;
    char ch;
    while (read(fd, &ch, 1) > 0) {
        buffer += ch;
        if (buffer.size() >= prompt.size() &&
            buffer.compare(buffer.size() - prompt.size(), prompt.size(), prompt) == 0) {
            if (captured_output)
                *captured_output = buffer;
            return true;
        }
    }
    return false;
}

/**
 * Rename formula via Maude CLI
 * E.g.,
 * Return a vector of string, the first one is the renamed formula, and
 * the rest are the renamed propositions.
 */
std::vector<std::string> renameFormula(const std::string& formula_str, 
        const std::string& spec_syntax_file_path) {
    int master_fd;
    pid_t pid = forkpty(&master_fd, nullptr, nullptr, nullptr);

    if (pid == 0) {
        // Child process: execute Maude
        execlp("maude", "maude", (char*)nullptr);
        perror("execlp failed");
        return {};
    }

    // Parent process
    if (!waitForPrompt(master_fd, "Maude>")) {
        std::cerr << "Failed to start Maude." << std::endl;
        return {};
    }
    sendCommand(master_fd, "load " + spec_syntax_file_path);

    if (!waitForPrompt(master_fd, "Maude>")) {
        std::cerr << "Failed to load model." << std::endl;
        return {};
    }

    // std::cout << "\n[Model loaded successfully]\n";
    sendCommand(master_fd, "red rename(" + formula_str + ") .");

    std::string reduction_output;
    std::vector<std::string> result = {};
    if (waitForPrompt(master_fd, "Maude>", &reduction_output)) {
        // std::cout << "\n=== Reduction Output ===\n";
        size_t pos = reduction_output.find("Maude>");
        if (pos != std::string::npos)
            reduction_output.erase(pos);
        // std::cout << reduction_output << std::endl;
        std::string result_str = extractResultFromOutput(reduction_output);
        if (result_str == "") {
            std::cerr << "A syntax error with formula: " << formula_str << std::endl;
            std::cerr << "Reduction output from Maude:\n" << reduction_output << std::endl;
        }
        else {
            result = toVectorString(result_str);
        }
        // std::cout << "\n[Extracted Result]:" << result << std::endl;
    } else {
        std::cerr << "No reduction result captured." << std::endl;
    }

    // Quit Maude
    sendCommand(master_fd, "quit");
    waitpid(pid, nullptr, 0);
    return result;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> mergePoints(
        const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& v1,
        size_t range,
        const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& v2) {
    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> result;
    result.insert(result.end(), v1.begin(), v1.begin() + std::min(range, v1.size()));
    result.insert(result.end(), v2.begin(), v2.end());
    return result;
}

/**
 * PlanningShield member methods
 */
PlanningShield::PlanningShield(
        rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_trajectory_publisher,
        rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_motion_velocity_publisher,
        std::string spec_formula_str, std::string spec_syntax_file_path,
        float speed_threshold_activation, float min_acc, float min_jerk)
    : verified_trajectory_publisher_(verified_trajectory_publisher),
      verified_motion_velocity_publisher_(verified_motion_velocity_publisher),
      speed_threshold_activation_(speed_threshold_activation), min_acc_(min_acc),
      min_jerk_(min_jerk), spec_syntax_file_path_(spec_syntax_file_path) {

    auto roots = solveRootPolynomial2(2.0, -3.0, 1.0);
    std::cout << "Test roots: ";
    for (const auto& r : roots) {
        std::cout << r << " ";
    }
    std::vector<std::string> renamed_spec_props = renameFormula(spec_formula_str, spec_syntax_file_path);
    if (renamed_spec_props.empty()) {
        throw std::runtime_error("Failed to parse the formula.");
    }
    std::string renamed_spec_formula_str = renamed_spec_props[0];

    for (size_t i = 1; i < renamed_spec_props.size(); ++i) {
        std::string& prop = renamed_spec_props[i];
        if (prop.find('$') != std::string::npos) {
            std::vector<std::string> tokens = splitString(prop, '$');

            if (prop.find('-') != std::string::npos) {
                // e.g., time-le-1.0e-1
                std::string old_str = prop;
                std::replace(prop.begin(), prop.end(), '-', '_');
                replaceSubStr(renamed_spec_formula_str, old_str, prop);
            }
            std::replace(prop.begin(), prop.end(), '$', '_');
            if (tokens[1] == "gt") {
                this->proposition_map_[prop] = [threshold = std::stof(tokens[2])](float val) {
                    return val > threshold;
                };
            }
            else if (tokens[1] == "ge") {
                this->proposition_map_[prop] = [threshold = std::stof(tokens[2])](float val) {
                    return val >= threshold;
                };
            }
            else if (tokens[1] == "lt") {
                this->proposition_map_[prop] = [threshold = std::stof(tokens[2])](float val) {
                    return val < threshold;
                };
            }
            else if (tokens[1] == "le") {
                this->proposition_map_[prop] = [threshold = std::stof(tokens[2])](float val) {
                    return val <= threshold;
                };
            }
            else if (tokens[1] == "eq") {
                this->proposition_map_[prop] = [threshold = std::stof(tokens[2])](float val) {
                    return std::abs(val - threshold) < 1e-6;
                };
            }
            else {
                std::cerr << "[WARNING] Unsupported comparison operator in proposition: " << prop << std::endl;
            }
        }
        this->propositions_.push_back(prop);
    }

    std::replace(renamed_spec_formula_str.begin(), renamed_spec_formula_str.end(), '$', '_');
    replaceSubStr(renamed_spec_formula_str, "\\\\", "\\");

    std::cout << "Spec formula: " << renamed_spec_formula_str << std::endl;
    for (const std::string& p : this->propositions_) {
        std::cout << "Proposition: " << p << std::endl;
    }
    std::cout << this->proposition_map_.size() << std::endl;
    for (const auto& [key, _] : this->proposition_map_) {
        std::cout << "Mapped proposition: " << key << std::endl;
    }

    try {
        this->spec_formula_ = spot::parse_infix_psl(renamed_spec_formula_str);
        if (this->spec_formula_.format_errors(std::cerr))
            std::cerr << "[ERROR] Failed to parse specification formula: " << renamed_spec_formula_str << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse specification formula: " << e.what() << std::endl;
    }
}

void PlanningShield::intervene(const autoware_planning_msgs::msg::Trajectory& planning_msg, 
        const nlohmann::json& recorded_data) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (recorded_data[PerceptionObjectTopic::TRACE_KEY()].empty() ||
        recorded_data[EstimatedKinematicTopic::TRACE_KEY()].empty() ||
        !recorded_data.contains(GroundtruthSizeTopic::TRACE_KEY())) {
        this->verified_trajectory_publisher_->publish(planning_msg);
        return;
    }
    // cache ego shape
    if (this->ego_shape_.empty()) {
        auto gtsize = recorded_data[GroundtruthSizeTopic::TRACE_KEY()];
        if (!gtsize.contains("vehicle_sizes")) {
            this->verified_trajectory_publisher_->publish(planning_msg);
            return;
        }
        for (const auto& entry : gtsize["vehicle_sizes"]) {
            if (entry["name"] == "ego") {
                this->ego_shape_ = entry;
                break;
            }
        }
        if (this->ego_shape_.empty()) {
            this->verified_trajectory_publisher_->publish(planning_msg);
            return;
        }
    }
    double current_time = timestamp(planning_msg.header.stamp);
    bool safe = true;
    if (getCurrentSpeed(recorded_data) >= this->speed_threshold_activation_) {
        const nlohmann::json& perception_msgs = recorded_data[PerceptionObjectTopic::TRACE_KEY()];
        const nlohmann::json& estimated_kinematic = recorded_data[EstimatedKinematicTopic::TRACE_KEY()].back();

        // verify the safety
        PlanningTrajectory planning_points = this->toPlanningTrajectoryObj(planning_msg, estimated_kinematic,this->ego_shape_);
        safe = this->verify(planning_points, perception_msgs);
        if (!safe) {
            std::cout << "Unsafe planning trajectory. Replace with a safe one..." << std::endl;
            std::vector<autoware_planning_msgs::msg::TrajectoryPoint> new_trajectory_points = 
                this->resampleTrajectoryPoints(
                    planning_msg, 
                    planning_points.start_point_id, 
                    perception_msgs,
                    planning_points.ego_current_state);
            autoware_planning_msgs::msg::Trajectory new_trajectory_msg;
            new_trajectory_msg.header = planning_msg.header;
            new_trajectory_msg.points = new_trajectory_points;

            this->verified_trajectory_publisher_->publish(new_trajectory_msg);

            auto end_time =  std::chrono::high_resolution_clock::now();
            verification_times_.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

            // validate in the last planning point, the vehicle's speed is 0
            if (std::abs(new_trajectory_points.back().longitudinal_velocity_mps) > 1e-4) {
                std::cerr << "[ERROR] The last point of revised trajectory does not have speed 0" << std::endl;
            }
            this->stop_point_ = extractObjPosition(new_trajectory_points.back());
            this->has_stop_point_ = true;
            this->last_stop_time_ = current_time;
        }
    } 
    if (safe) {
        // if safe, publish the trajectory
        this->verified_trajectory_publisher_->publish(planning_msg);
        if (current_time >= this->last_stop_time_ + 4.0) {
            // reset
            std::cout << "Restart moving..." << std::endl;
            this->has_stop_point_ = false;
            this->last_stop_time_ = 1e20;
        }
    }
}

void PlanningShield::interveneMotionVelocityMsg(const autoware_planning_msgs::msg::Trajectory& trajectory_msg) {
    if (this->has_stop_point_) {
        autoware_planning_msgs::msg::Trajectory new_msg = 
            injectMotionVelocityTrajectory(trajectory_msg, this->stop_point_);
        this->verified_motion_velocity_publisher_->publish(new_msg);
    } else {
        this->verified_motion_velocity_publisher_->publish(trajectory_msg);
    }
}

bool PlanningShield::verify(const PlanningTrajectory& planning_points, 
        const nlohmann::json& perception_msgs){
    auto latest_perception_msg = perception_msgs.back();
    if (latest_perception_msg["objects"].empty()) {
        return true;
    }

    for (const auto& perceived_obj : latest_perception_msg["objects"]) {
        // perceived object dimension
        std::vector<glm::vec2> npc_local_vertices;
        if (perceived_obj["shape"]["type"] == "box") {
            float length = perceived_obj["shape"]["size"]["x"];
            float width = perceived_obj["shape"]["size"]["y"];
            npc_local_vertices = std::vector<glm::vec2> {
                glm::vec2(width / 2, length / 2),
                glm::vec2(-width / 2, length / 2),
                glm::vec2(-width / 2, -length / 2),
                glm::vec2(width / 2, -length / 2),
            };
        } else if (perceived_obj["shape"]["type"] == "polygon") {
            for (const auto& point : perceived_obj["shape"]["footprint"]) {
                npc_local_vertices.push_back(glm::vec2(point["x"], point["y"]));
            }
        }
        auto current_pose = perceived_obj["pose"];
        float current_heading = current_pose["rotation"]["z"];
        auto current_vel = jsonPointToVector2(perceived_obj["twist"]["linear"]);
        auto current_position = jsonPointToVector2(current_pose["position"]);
        float existence_prob = perceived_obj["existence_prob"];
        if (!this->verifySimulatedNpcPath(planning_points,
                            existence_prob,
                            current_position, current_heading,
                            current_vel, npc_local_vertices)) {
            return false;
        }

        for (size_t i = 0; i < perceived_obj["predict_paths"].size(); ++i) {
            const auto& predicted_path = perceived_obj["predict_paths"][i];
            // if (predicted_path['confidence'] >= 0.1) {
            if (!this->verifyPredictNpcPath(planning_points,
                            existence_prob, predicted_path,
                            current_heading, npc_local_vertices)) {
                return false;
            }
            // }
        }
    }
    return true;
}

bool PlanningShield::verifySimulatedNpcPath(const PlanningTrajectory& planning_points,
                                float existence_prob, const glm::vec2& current_position, float current_heading,
                                const glm::vec2& current_vel, const std::vector<glm::vec2>& npc_local_vertices,
                                float time_bound) {
    /**
     * Assume the NPC keep going with given (current) velocity.
     * Simulate the path and TODO
     */
    std::vector<float> time_series;
    std::vector<bool> collision_series;
    std::vector<float> distance_series;
    float last_time = -10.0;
    for (const auto& ego_point : planning_points.future_points) {
        // Ego
        float time_from_start = ego_point.time_from_start;
        if (time_from_start > time_bound) {
            break;
        }

        bool skip_cond1 = time_from_start > 0.0 &&
                        time_from_start <= 1.0 &&
                        time_from_start - last_time <= 0.5;
        bool skip_cond2 = time_from_start >= 1.0 &&
                        time_from_start - last_time <= 0.2;

        if (skip_cond1 || skip_cond2) {
            continue;
        }

        glm::vec2 planning_position = ego_point.position;
        std::vector<glm::vec2> ego_plan_vertices = (
            planning_points.ego_current_state.getVerticesWhenAtPosition(planning_position));
        time_series.push_back(time_from_start);

        // NPC
        glm::vec2 npc_position = current_position + current_vel * time_from_start;
        std::vector<glm::vec2> npc_vertices = getObjectWorldVertices(
            npc_position,
            current_heading,
            npc_local_vertices
        );
        collision_series.push_back(isCollision(ego_plan_vertices, npc_vertices));
        distance_series.push_back(glm::length(planning_position - npc_position));
    }
    // data_series = {
    //     'time': time_series,
    //     'collision': collision_series,
    //     'distance': distance_series,
    //     'path_confidence': [1.0],
    //     'existence_prob': [existence_prob],
    // }
    return this->evaluateSpec(time_series, collision_series, distance_series, existence_prob, 1.0);
}

bool PlanningShield::verifyPredictNpcPath(const PlanningTrajectory& planning_points,
                              float existence_prob, const nlohmann::json& predicted_path, float current_heading, const std::vector<glm::vec2>& npc_local_vertices,
                              float time_bound) {
    /**
    * Construct the transitions from a predicted path.
    */
    float npc_path_time_step = predicted_path["time_step"];
    if (planning_points.future_points.empty()) {
        return true;
    }
    if (npc_path_time_step < planning_points.future_points[0].time_to_next_point) {
        std::cout << "[ERROR] Not handled NPC path time step smaller than "
                  << planning_points.future_points[0].time_to_next_point << std::endl;
        return true;
    }

    std::vector<float> time_series;
    std::vector<bool> collision_series;
    std::vector<float> distance_series;
    float last_time = -10.0;
    for (const auto& ego_point : planning_points.future_points) {
        float time_from_start = ego_point.time_from_start;
        if (time_from_start > time_bound) {
            break;
        }

        bool skip_cond1 = time_from_start > 0.0 &&
                        time_from_start <= 1.0 &&
                        time_from_start - last_time <= 0.5;
        bool skip_cond2 = time_from_start >= 1.0 &&
                        time_from_start - last_time <= 0.2;

        if (skip_cond1 || skip_cond2) {
            continue;
        }

        glm::vec2 planning_position = ego_point.position;
        std::vector<glm::vec2> ego_plan_vertices = (
            planning_points.ego_current_state.getVerticesWhenAtPosition(planning_position));

        size_t no_npc_path_time_step = static_cast<size_t>(time_from_start / npc_path_time_step);
        float remain_time = time_from_start - no_npc_path_time_step * npc_path_time_step;

        if (no_npc_path_time_step >= predicted_path["path"].size() - 2) {
            break;
        }
        const auto& npc_point0 = predicted_path["path"][no_npc_path_time_step];
        const auto& npc_point1 = predicted_path["path"][no_npc_path_time_step + 1];

        glm::vec2 npc_pos0 = glm::vec2(npc_point0["position"]["x"], npc_point0["position"]["y"]);
        glm::vec2 npc_pos1 = glm::vec2(npc_point1["position"]["x"], npc_point1["position"]["y"]);
        glm::vec2 segment = npc_pos1 - npc_pos0;
        glm::vec2 estimated_npc_pos = npc_pos0 + remain_time / npc_path_time_step * segment;

        std::vector<glm::vec2> npc_vertices = getObjectWorldVertices(
            estimated_npc_pos,
            current_heading,
            npc_local_vertices
        );
        collision_series.push_back(isCollision(ego_plan_vertices, npc_vertices));
        distance_series.push_back(glm::length(planning_position - estimated_npc_pos));
    }

    // data_series = {
    //     'time': time_series,
    //     'collision': collision_series,
    //     'distance': distance_series,
    //     'path_confidence': [predicted_path['confidence']],
    //     'existence_prob': [existence_prob],
    // }
    float path_confidence = predicted_path["confidence"];
    return this->evaluateSpec(time_series, collision_series, distance_series, existence_prob, path_confidence);
}

// Resample the trajectory to guarantee the safety requirements
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> PlanningShield::resampleTrajectoryPoints(
        const autoware_planning_msgs::msg::Trajectory& trajectory_msg, 
        size_t _id, 
        const nlohmann::json& perception_msgs, 
        const ConstantHeadingVehicle& ego_current_state) {
    float jerk = -6.0;
    float acc = -3.0;
    while (true) {
        std::vector<autoware_planning_msgs::msg::TrajectoryPoint> new_ahead_points = 
            resampleTrajectory(trajectory_msg.points, _id, -jerk, acc);

        PlanningTrajectory new_planning_points = PlanningTrajectory(ego_current_state);
        new_planning_points.parseFuturePoints(new_ahead_points);

        if (this->verify(new_planning_points, perception_msgs)) {
            // this new trajectory is safe
            std::cout << "Braking profile acc: " << acc << ", jerk: " << jerk << " was applied to revise trajectory." << std::endl;
            return mergePoints(trajectory_msg.points, _id, new_ahead_points);
        }
        jerk -= 2.0;
        acc -= 1.0;
        if (jerk < this->min_jerk_ || acc < this->min_acc_) {
            // Cannot find a trajectory that makes no collision
            // Apply the strongest braking profile
            std::cout << "Maximum braking profile was applied in the revised trajectory." << std::endl;
            return mergePoints(trajectory_msg.points, _id, new_ahead_points);
        }
    }
}

/**
 * evaluate the safety specification by constructing a Kripke structure 
 * and checking the formula with Spot.
 */
bool PlanningShield::evaluateSpec(const std::vector<float>& time_series,
        const std::vector<bool>& collision_series,
        const std::vector<float>& distance_series,
        float existence_prob,
        float path_confidence) {
    spot::bdd_dict_ptr dict = spot::make_bdd_dict();
    spot::kripke_graph_ptr k = spot::make_kripke_graph(dict);

    std::vector<int> ap_ids;
    for (const auto& prop : this->propositions_) {
        ap_ids.push_back(k->register_ap(prop));
    }

    unsigned lastest_state_id = 0;
    for (size_t i = 0; i < time_series.size(); ++i) {
        bdd label = bddtrue;

        for (size_t j = 0; j < this->propositions_.size(); ++j) {
            const std::string& prop = this->propositions_[j];
            bool prop_holds = false;
            if (this->proposition_map_.find(prop) != this->proposition_map_.end()) {
                float val = 0.0;
                if (startsWith(prop, "time")) {
                    val = time_series[i];
                } else if (startsWith(prop, "distance")) {
                    val = distance_series[i];
                } else if (startsWith(prop, "existenceProb")) {
                    val = existence_prob;
                } else if (startsWith(prop, "pathConfidence")) {
                    val = path_confidence;
                } else {
                    std::cerr << "[WARNING] Unsupported proposition : " << prop << std::endl;
                    continue;
                }
                prop_holds = this->proposition_map_[prop](val);
            } else {
                if (prop == "collision") {
                    prop_holds = collision_series[i];
                } else {
                    std::cerr << "[WARNING] Unsupported proposition: " << prop << std::endl;
                    continue;
                }
            }
            if (prop_holds) {
                label &= bdd_ithvar(ap_ids[j]);
            } else {
                label &= bdd_nithvar(ap_ids[j]);
            }
        }

        // E.g., time-le-3.0
        // if (time_series[i] <= 3.0) {
        //     label &= bdd_ithvar(ap_ids[0]);
        // } else {
        //     label &= bdd_nithvar(ap_ids[0]);
        // }

        unsigned state_id = k->new_state(label);
        if (i == 0) {
            k->set_init_state(state_id);
        }
        else if (i > 0) {
            k->new_edge(lastest_state_id, state_id);
        }
        lastest_state_id = state_id;
    }
    if (lastest_state_id == 0)
        return true;

    k->new_edge(lastest_state_id, lastest_state_id); // self-loop for the last state

    // Translate formula negation.
    spot::formula f = spot::formula::Not(this->spec_formula_.f);
    spot::twa_graph_ptr af = spot::translator(dict).run(f);

    return !k->intersecting_run(af);
}