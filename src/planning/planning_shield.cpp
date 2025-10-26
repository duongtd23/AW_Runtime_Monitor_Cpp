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

// helper functions
glm::vec2 extractObjPosition(const autoware_planning_msgs::msg::TrajectoryPoint& entry) {
    return glm::vec2(entry.pose.position.x, entry.pose.position.y);
}

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

PlanningShield::PlanningShield(std::string spec_formula_str, std::vector<std::string> propositions, 
        float speed_threshold_activation, float min_acc, float min_jerk)
    : speed_threshold_activation_(speed_threshold_activation), min_acc_(min_acc), min_jerk_(min_jerk) {
    for (std::string& prop : propositions) {
        if (prop.find('$') != std::string::npos) {
            std::vector<std::string> tokens = splitString(prop, '$');
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

        if (prop.find('-') != std::string::npos) {
            // e.g., time-le-1.0e-1
            std::string old_str = prop;
            std::replace(prop.begin(), prop.end(), '-', '_');
            replaceSubStr(spec_formula_str, old_str, prop);
        }
        std::replace(prop.begin(), prop.end(), '$', '_');
        this->propositions_.push_back(prop);
    }

    std::replace(spec_formula_str.begin(), spec_formula_str.end(), '$', '_');
    std::cout << spec_formula_str << std::endl;
    for (const std::string& p : this->propositions_) {
        std::cout << "Proposition: " << p << std::endl;
    }

    try {
        this->spec_formula_ = spot::parse_infix_psl(spec_formula_str);
        if (this->spec_formula_.format_errors(std::cerr))
            std::cerr << "[ERROR] Failed to parse specification formula: " << spec_formula_str << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse specification formula: " << e.what() << std::endl;
    }
}

bool PlanningShield::verify(const autoware_planning_msgs::msg::Trajectory& trajectory_msg, const nlohmann::json& recorded_data){
    auto start_time = std::chrono::high_resolution_clock::now();

    if (recorded_data[PerceptionObjectTopic::TRACE_KEY()].empty() ||
        recorded_data[EstimatedKinematicTopic::TRACE_KEY()].empty() ||
        !recorded_data.contains(GroundtruthSizeTopic::TRACE_KEY())) {
        return true;
    }

    // cache ego shape
    if (this->ego_shape_.empty()) {
        auto gtsize = recorded_data[GroundtruthSizeTopic::TRACE_KEY()];
        if (!gtsize.contains("vehicle_sizes")) {
            return true;
        }
        for (const auto& entry : gtsize["vehicle_sizes"]) {
            if (entry["name"] == "ego") {
                this->ego_shape_ = entry;
                break;
            }
        }
        if (this->ego_shape_.empty()) {
            return true;
        }
    }
    bool safe = this->verify(trajectory_msg,
                    recorded_data[PerceptionObjectTopic::TRACE_KEY()],
                    recorded_data[EstimatedKinematicTopic::TRACE_KEY()].back(),
                    this->ego_shape_);
    if (!safe) {
        auto end_time =  std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Time: " << duration << " ms" << std::endl;
    }
    return safe;
}

bool PlanningShield::verify(const autoware_planning_msgs::msg::Trajectory& trajectory_msg, const nlohmann::json& perception_msgs, const nlohmann::json& estimated_kinematic, const nlohmann::json& ego_shape){
    auto latest_perception_msg = perception_msgs.back();
    if (latest_perception_msg["objects"].empty()) {
        return true;
    }
    // std::cout << "Debug 3" << std::endl;
    PlanningTrajectory planning_points = this->toPlanningTrajectoryObj(trajectory_msg, estimated_kinematic, ego_shape);

    // std::cout << "Debug 4" << std::endl;

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
    // std::cout << latest_perception_msg.dump(2) << std::endl;
    // std::cout << estimated_kinematic.dump(2) << std::endl;
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
                    std::cerr << "[WARNING] Unsupported proposition: " << prop << std::endl;
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

    bool result = !k->intersecting_run(af);
    // if (auto run = k->intersecting_run(af))
    //     result = false;
    // else
    //     result = true;

    bool verified = true;
    for (size_t i = 0; i < time_series.size(); ++i) {
        if (time_series[i] > 3.0) {
            break;
        }
        if (collision_series[i] && path_confidence >= 0.1) {
            verified = false;
            break;
        }
    }
    if (result != verified) {
        std::cout << "[ERROR] Spot verification result inconsistent with direct evaluation!" << std::endl;
    }
    return result;
}