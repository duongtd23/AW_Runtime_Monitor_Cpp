#include <string>
#include <set>
#include "aw_runtime_monitor/planning/planning_shield.hpp"
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "rclcpp/serialization.hpp"
#include "aw_runtime_monitor/utils.hpp"

#include "autoware/route_handler/route_handler.hpp"

/**
 * PlanningShield member methods
 */
PlanningShield::PlanningShield(
        const std::string& spec_formula_str, const std::string& spec_syntax_file_path,
        float speed_threshold_activation, float time_bound, float distance_bound,
        const RevisionConfig& revision_config, bool enable_revision)
    : spec_syntax_file_path_(spec_syntax_file_path),
      speed_threshold_activation_(speed_threshold_activation), 
      time_bound_(time_bound), distance_bound_(distance_bound), enable_revision_(enable_revision),
      logger_(rclcpp::get_logger("planning_shield")) {

    drivable_area_checker_ = ExternalDrivableAreaChecker();
    cached_bdd_dict_ = spot::make_bdd_dict();
    trajectory_reviser_ = TrajectoryReviser(revision_config, logger_);

    // the first element is the renamed formula, the rest are the renamed propositions
    std::vector<std::string> renamed_spec_props = renameFormula(spec_formula_str, spec_syntax_file_path);
    if (renamed_spec_props.empty()) {
        throw std::runtime_error("Failed to parse the formula.");
    }
    std::string renamed_spec_formula_str = renamed_spec_props[0];

    for (size_t i = 1; i < renamed_spec_props.size(); ++i) {
        std::string& prop = renamed_spec_props[i];
        if (prop.find('_') != std::string::npos) {
            std::vector<std::string> tokens = splitString(prop, '_');

            if (prop.find('-') != std::string::npos) {
                // e.g., time_le_1.0e-1
                std::string old_str = prop;
                std::replace(prop.begin(), prop.end(), '-', '_');
                replaceSubStr(renamed_spec_formula_str, old_str, prop);
            }
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

    replaceSubStr(renamed_spec_formula_str, "\\\\", "\\");

    // std::cout << "Planning monitor is enabled, formula: " << renamed_spec_formula_str << std::endl;
    // for (const std::string& p : this->propositions_) {
    //     std::cout << "Proposition: " << p << std::endl;
    // }

    try {
        this->spec_formula_ = spot::parse_infix_psl(renamed_spec_formula_str);
        if (this->spec_formula_.format_errors(std::cerr))
            std::cerr << "[ERROR] Failed to parse specification formula: " << renamed_spec_formula_str << std::endl;

        bdd_dict_ = spot::make_bdd_dict();
        kripke_graph_ = spot::make_kripke_graph(bdd_dict_);
        for (const auto& prop : this->propositions_) {
            ap_ids_.push_back(kripke_graph_->register_ap(prop));
        }
        // Translate formula negation.
        spot::formula f = spot::formula::Not(this->spec_formula_.f);
        af_ = spot::translator(bdd_dict_).run(f);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse specification formula: " << e.what() << std::endl;
    }
}

/**
 * @brief Verify the planned trajectory against the safety specification, using the detected objects from recorded data
 * @param planning_msg The planned trajectory message
 * @param recorded_data The recorded data containing estimated kinematic and perception objects
 * @return true if the trajectory is safe, false otherwise
 */
PlanningShield::VerificationResult PlanningShield::verify(const autoware_planning_msgs::msg::Trajectory& planning_msg, 
        const nlohmann::json& recorded_data) {
    // auto start_time = std::chrono::high_resolution_clock::now();
    
    if (recorded_data.find(EstimatedKinematicTopic::TRACE_KEY()) == recorded_data.end() ||
        recorded_data.at(EstimatedKinematicTopic::TRACE_KEY()).empty() ||
        recorded_data.find(PerceptionObjectTopic::TRACE_KEY()) == recorded_data.end() ||
        recorded_data.at(PerceptionObjectTopic::TRACE_KEY()).empty() ||
        !recorded_data.contains(GroundtruthSizeTopic::TRACE_KEY())) {
            latest_verification_result_ = {true, false, planning_msg};
            return {true, false, planning_msg}; // assume safe if no data
    }
    const nlohmann::json& estimated_kinematic = recorded_data[EstimatedKinematicTopic::TRACE_KEY()].back();
    auto current_pose = estimated_kinematic["pose"];
    glm::vec3 current_position = jsonPointToVector3(current_pose["position"]);
    // glm::vec3 current_eulerangles = jsonPointToVector3(current_pose["rotation"]); // angles in degrees
    // std::cout << "[DEBUG] Current Ego Position: (" 
    //           << current_position.x << ", " 
    //           << current_position.y << ", " 
    //           << current_position.z << ")" << std::endl;

    auto id_dis = extractClosestPoint(planning_msg.points, glm::vec2(current_position.x, current_position.y));

    // only consider trajectory points ahead of the ego vehicle's current position
    size_t closest_point_id = std::get<0>(id_dis);

    // Extract Ego shape
    float ego_length = 0.0f, ego_width = 0.0f, ego_center_x = 0.0f, ego_center_y = 0.0f;
    auto gtsize = recorded_data[GroundtruthSizeTopic::TRACE_KEY()];
    for (const auto& entry : gtsize["vehicle_sizes"]) {
        if (entry["name"] == "ego") {
            ego_length = entry["size"]["x"];
            ego_width = entry["size"]["y"];
            ego_center_x = entry["center"]["x"];
            ego_center_y = entry["center"]["y"];
            break;
        }
    }
    glm::vec2 ego_size(ego_length, ego_width);
    glm::vec2 ego_center_offset(ego_center_x, ego_center_y);

    auto internal_result = doVerify(
        planning_msg,
        recorded_data,
        closest_point_id,
        ego_size,
        ego_center_offset);
    
    bool is_safe = internal_result.is_safe;
    std::vector<size_t> collision_indices = internal_result.collision_indices;
    
    if (is_safe) {
        latest_verification_result_ = {true, false, planning_msg};
        return {true, false, planning_msg};
    }
    
    // Violation found - attempt trajectory revision
    RCLCPP_INFO(logger_, "Safety violation detected with %zu collision points.", collision_indices.size());

    if (!enable_revision_) {
        RCLCPP_INFO(logger_, "Revision disabled. Returning original trajectory.");
        latest_verification_result_ = {false, false, planning_msg};
        return {false, false, planning_msg};
    }

    // Cache drivable area for the current pose
    geometry_msgs::msg::Pose ego_pose;
    ego_pose.position.x = current_position.x;
    ego_pose.position.y = current_position.y;
    ego_pose.position.z = current_position.z;
    drivable_area_checker_.cacheCurrentDrivableArea(ego_pose);

    // Create verify function for the reviser
    auto verify_func = [this, &recorded_data, closest_point_id, &ego_size, &ego_center_offset](const autoware_planning_msgs::msg::Trajectory& traj) -> bool {
        // Re-verify the trajectory
        auto result = this->doVerify(traj, recorded_data, closest_point_id, ego_size, ego_center_offset);
        return result.is_safe;
    };

    // Attempt revision
    RevisionResult revision_result = trajectory_reviser_.revise(
        planning_msg,
        collision_indices,
        drivable_area_checker_,
        verify_func,
        closest_point_id,
        ego_size,
        ego_center_offset
    );
    
    if (revision_result.success) {
        RCLCPP_INFO(logger_, "Successfully revised trajectory (%s)", revision_result.applied_candidate.toString().c_str());
        latest_verification_result_ = {false, true, revision_result.trajectory};
        return {false, true, revision_result.trajectory};
    } else {
        // RCLCPP_ERROR(logger_, "Failed to find safe trajectory revision");
        latest_verification_result_ = {false, false, planning_msg};
        return {false, false, planning_msg};
    }
}

/**
 * @brief Internal verification that returns collision information
 */
PlanningShield::InternalVerificationResult PlanningShield::doVerify(
        const autoware_planning_msgs::msg::Trajectory& planning_msg,
        const nlohmann::json& recorded_data,
        size_t closest_point_id,
        // const glm::vec3& current_position, // current ego position
        const glm::vec2& ego_size,
        const glm::vec2& ego_center_offset) {

    // Compute timestamps for each trajectory point (starting from closest_point_id)
    std::vector<float> trajectory_timestamps;
    std::vector<glm::vec2> trajectory_positions;
    std::vector<float> trajectory_headings;  // in degrees
    std::vector<float> trajectory_z_values;

    float cumulative_time = 0.0f;
    for (size_t i = closest_point_id; i < planning_msg.points.size(); ++i) {
        const auto& point = planning_msg.points[i];
        
        if (i > closest_point_id) {
            const auto& prev_point = planning_msg.points[i - 1];
            glm::vec2 pos_prev = extractObjPosition(prev_point);
            glm::vec2 pos_curr = extractObjPosition(point);
            float speed_prev = std::sqrt(prev_point.longitudinal_velocity_mps * prev_point.longitudinal_velocity_mps +
                                         prev_point.lateral_velocity_mps * prev_point.lateral_velocity_mps);
            float speed_curr = std::sqrt(point.longitudinal_velocity_mps * point.longitudinal_velocity_mps +
                                         point.lateral_velocity_mps * point.lateral_velocity_mps);
            if (speed_prev < 0.01) {
                break;
            }
            float accel_prev = prev_point.acceleration_mps2;
            float accel_curr = point.acceleration_mps2;
            
            float dt = deriveTimeStep(pos_prev, speed_prev, accel_prev, pos_curr, speed_curr, accel_curr);
            cumulative_time += dt;
        }
        
        // Stop if beyond time bound
        if (cumulative_time > time_bound_) {
            break;
        }
        
        trajectory_timestamps.push_back(cumulative_time);
        trajectory_positions.push_back(extractObjPosition(point));
        trajectory_z_values.push_back(point.pose.position.z);
        
        // Extract heading (yaw) from quaternion
        auto euler = quaternionToEulerAngles(point.pose.orientation);
        trajectory_headings.push_back(static_cast<float>(euler[2]));  // yaw in degrees
    }

    if (trajectory_positions.empty()) {
        return {true, {}}; // no trajectory points to verify
    }

    // Get the latest perception data
    const nlohmann::json& perception_data = recorded_data[PerceptionObjectTopic::TRACE_KEY()].back();

    std::set<size_t> collision_index_set;
    bool is_safe = true;
    
    // Iterate over each detected object
    for (const auto& obj : perception_data["objects"]) {
        float existence_prob = obj["existence_prob"];
        
        // Get object's current position and check initial distance
        glm::vec2 obj_current_pos = jsonPointToVector2(obj["pose"]["position"]);
        // float obj_z = obj["pose"]["position"]["z"];
        float initial_distance = glm::length(trajectory_positions[0] - obj_current_pos);
        
        // Skip objects that are too far away
        if (initial_distance > distance_bound_) {
            continue;
        }
        
        // Get object heading
        // float obj_heading = obj["pose"]["rotation"]["z"];  // yaw in degrees
        
        // Get object's local vertices based on shape
        std::vector<glm::vec2> obj_local_vertices;
        if (obj["shape"]["type"] == "box") {
            float length = obj["shape"]["size"]["x"];
            float width = obj["shape"]["size"]["y"];
            obj_local_vertices = {
                glm::vec2(length / 2, width / 2),
                glm::vec2(-length / 2, width / 2),
                glm::vec2(-length / 2, -width / 2),
                glm::vec2(length / 2, -width / 2),
            };
        } else if (obj["shape"]["type"] == "polygon") {
            for (const auto& pt : obj["shape"]["footprint"]) {
                obj_local_vertices.push_back(glm::vec2(pt["x"], pt["y"]));
            }
        } else {
            // Cylinder or unsupported shape - skip
            continue;
        }

        // Get object's current heading (kept constant for simulated path)
        // float obj_heading = obj["pose"]["rotation"]["z"];  // yaw in degrees
        // float obj_z = obj["pose"]["position"]["z"];

        // Compute velocity from historical data for simulated path
        // float current_time = perception_data["timestamp"];
        // std::string npc_id = obj["id"];
        // glm::vec2 cal_vel = correctVelocity(recorded_data[PerceptionObjectTopic::TRACE_KEY()],
        //                             obj_current_pos,
        //                             current_time,
        //                             npc_id,
        //                             recorded_data[PerceptionObjectTopic::TRACE_KEY()].size() - 1,
        //                             1);
        
        // // Verify with simulated path (constant velocity assumption)
        // // Only check if we have valid velocity data (non-zero)
        // if (cal_vel.x != 0.0f || cal_vel.y != 0.0f) {
        //     std::vector<float> sim_timestamp_series;
        //     std::vector<bool> sim_collision_series;
        //     std::vector<float> sim_distance_series;
        //     // Track collision information for potential trajectory revision
        //     std::vector<size_t> collision_indices;
            
        //     for (size_t i = 0; i < trajectory_timestamps.size(); ++i) {
        //         float t = trajectory_timestamps[i];
        //         glm::vec2 ego_pos = trajectory_positions[i];
        //         float ego_heading = trajectory_headings[i];
        //         float ego_z = trajectory_z_values[i];
                
        //         // Simulate NPC position assuming constant velocity
        //         glm::vec2 npc_pos = obj_current_pos + cal_vel * t;
                
        //         // Compute distance
        //         float distance = glm::length(ego_pos - npc_pos);
                
        //         // Check z-value difference (only consider collision if on same level)
        //         bool same_z_level = std::abs(ego_z - obj_z) < 3.0f;
                
        //         // Compute collision
        //         bool collision = false;
        //         if (same_z_level) {
        //             std::vector<glm::vec2> ego_vertices = getEgoWorldVertices(
        //                 ego_pos, ego_heading, ego_size, ego_center_offset);
        //             std::vector<glm::vec2> npc_vertices = getObjectWorldVertices(
        //                 npc_pos, obj_heading, obj_local_vertices);
        //             collision = isCollision(ego_vertices, npc_vertices);
        //         }
                
        //         sim_timestamp_series.push_back(t);
        //         sim_collision_series.push_back(collision);
        //         sim_distance_series.push_back(distance);
                
        //         // Track collision indices for trajectory revision
        //         if (collision) {
        //             collision_indices.push_back(i + closest_point_id);
        //         }
        //     }
            
        //     // Verify safety specification for simulated path (use confidence = 1.0 for simulated path)
        //     if (!evaluateSpec(sim_timestamp_series, sim_collision_series, sim_distance_series, existence_prob, 1.0f)) {
        //         return {false, collision_indices};
        //     }
        // }
        
        // Iterate over each predicted path of this object
        for (const auto& predicted_path : obj["predict_paths"]) {
            float path_confidence = predicted_path["confidence"];
            float npc_time_step = predicted_path["time_step"];
            const auto& path_points = predicted_path["path"];
            
            if (path_points.empty()) {
                continue;
            }
            
            std::vector<float> timestamp_series;
            std::vector<bool> collision_series;
            std::vector<float> distance_series;
            // Track collision information for potential trajectory revision
            // std::vector<size_t> collision_indices;
            
            // For each ego trajectory point, interpolate object position and check collision
            for (size_t i = 0; i < trajectory_timestamps.size(); ++i) {
                float t = trajectory_timestamps[i];
                glm::vec2 ego_pos = trajectory_positions[i];
                float ego_heading = trajectory_headings[i];
                float ego_z = trajectory_z_values[i];
                
                // Compute which segment of the predicted path corresponds to time t
                size_t path_index = static_cast<size_t>(t / npc_time_step);
                float remainder_time = t - path_index * npc_time_step;
                
                // Check if we have enough path points
                if (path_index >= path_points.size() - 1) {
                    // Use the last path point
                    path_index = path_points.size() - 2;
                    if (path_points.size() < 2) {
                        break;
                    }
                }
                
                // Interpolate object position
                const auto& npc_point0 = path_points[path_index];
                const auto& npc_point1 = path_points[path_index + 1];
                
                glm::vec2 npc_pos0 = jsonPointToVector2(npc_point0["position"]);
                glm::vec2 npc_pos1 = jsonPointToVector2(npc_point1["position"]);
                float npc_z0 = npc_point0["position"]["z"];
                float npc_z1 = npc_point1["position"]["z"];
                
                float interp_factor = remainder_time / npc_time_step;
                glm::vec2 npc_pos = npc_pos0 + interp_factor * (npc_pos1 - npc_pos0);
                float npc_z = npc_z0 + interp_factor * (npc_z1 - npc_z0);
                
                // Interpolate object heading
                float npc_heading0 = npc_point0["rotation"]["z"];
                float npc_heading1 = npc_point1["rotation"]["z"];
                float npc_heading = npc_heading0 + interp_factor * (npc_heading1 - npc_heading0);
                
                // Compute distance
                float distance = glm::length(ego_pos - npc_pos);
                
                // Check z-value difference (only consider collision if on same level)
                bool same_z_level = std::abs(ego_z - npc_z) < 3.0f;
                
                // Compute collision
                bool collision = false;
                if (same_z_level) {
                    std::vector<glm::vec2> ego_vertices = getEgoWorldVertices(
                        ego_pos, ego_heading, ego_size, ego_center_offset);
                    std::vector<glm::vec2> npc_vertices = getObjectWorldVertices(
                        npc_pos, npc_heading, obj_local_vertices);
                    collision = isCollision(ego_vertices, npc_vertices);
                }
                
                timestamp_series.push_back(t);
                collision_series.push_back(collision);
                distance_series.push_back(distance);
                
                // Track collision indices for trajectory revision
                if (collision) {
                    // collision_indices.push_back(i + closest_point_id);
                    collision_index_set.insert(i + closest_point_id);
                }
            }
            
            // Verify safety specification
            if (!evaluateSpec(timestamp_series, collision_series, distance_series, existence_prob, path_confidence)) {
                // std::cout << "[DEBUG] Object Predicted Path Points (x,y):" << std::endl;
                // for (const auto& pt : path_points) {
                //     std::cout << "  (" << pt["position"]["x"] << ", " << pt["position"]["y"] << "), " << std::endl;
                // }
                // return {false, collision_indices};
                is_safe = false;
            }
        }
    }

    std::vector<size_t> collision_indices(collision_index_set.begin(), collision_index_set.end());
    return {is_safe, collision_indices};
}

std::optional<autoware_planning_msgs::msg::Trajectory> PlanningShield::handleScenarioPlanningTrajectory(
        const autoware_planning_msgs::msg::Trajectory& scenario_planning_trajectory,
        const nlohmann::json& /*recorded_data*/) {

    // Update the scenario planning trajectory speed
    if (scenario_planning_trajectory.points.size() >= 1) {
        auto speed = scenario_planning_trajectory.points[0].longitudinal_velocity_mps;
        if (speed > 0.0) {
            scenario_planning_trajectory_speed = speed;
        }
    }

    // if the latest trajectory was revised,
    // update the speed of the scenario planning trajectory point
    if (latest_verification_result_.was_revised) {
        autoware_planning_msgs::msg::Trajectory updated_trajectory = latest_verification_result_.revised_msg;
        updated_trajectory.header.stamp = scenario_planning_trajectory.header.stamp;
        updated_trajectory.header.frame_id = scenario_planning_trajectory.header.frame_id;

        for (size_t i = 0; i < updated_trajectory.points.size(); ++i) {
            updated_trajectory.points[i].longitudinal_velocity_mps = scenario_planning_trajectory_speed;
            // other fields are irrelevant for scenario planning trajectory
            updated_trajectory.points[i].time_from_start.sec = 0;
            updated_trajectory.points[i].time_from_start.nanosec = 0;
            updated_trajectory.points[i].lateral_velocity_mps = 0.0;
            updated_trajectory.points[i].acceleration_mps2 = 0.0;
            updated_trajectory.points[i].heading_rate_rps = 0.0;
            updated_trajectory.points[i].front_wheel_angle_rad = 0.0;
            updated_trajectory.points[i].rear_wheel_angle_rad = 0.0;
        }
        return updated_trajectory;
    }

    return std::nullopt;
}

/**
 * @brief Evaluate the safety specification by constructing a Kripke structure
 * and checking the formula with Spot.
 * @param timestamp_series Series of timestamps associated with states of "the state machine"
 * @param collision_series Series of collision booleans, i.e., collision atomic proposition evaluation values
 * @param distance_series Series of distance values, used to derive distance-related atomic proposition evaluation values
 * @param existence_prob Probability of the object's existence
 * @param path_confidence Confidence of the object predicted travel path
 */
bool PlanningShield::evaluateSpec(const std::vector<float>& timestamp_series,
        const std::vector<bool>& collision_series,
        const std::vector<float>& distance_series,
        float existence_prob,
        float path_confidence) {
    
    if (timestamp_series.size() <= 1) {
        return true;
    }
    for (size_t i = 0; i < timestamp_series.size(); ++i) {
        bdd label = bddtrue;

        for (size_t j = 0; j < this->propositions_.size(); ++j) {
            const std::string& prop = this->propositions_[j];
            bool prop_holds = false;
            if (this->proposition_map_.find(prop) != this->proposition_map_.end()) {
                float val = 0.0;
                if (startsWith(prop, "time")) {
                    val = timestamp_series[i];
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
                label &= bdd_ithvar(ap_ids_[j]);
            } else {
                label &= bdd_nithvar(ap_ids_[j]);
            }
        }

        unsigned state_id = kripke_graph_->new_state(label);
        if (i == 0) {
            kripke_graph_->set_init_state(state_id);
        }
        else {
            kripke_graph_->new_edge(latest_state_id_, state_id);
        }
        latest_state_id_ = state_id;
    }

    kripke_graph_->new_edge(latest_state_id_, latest_state_id_); // self-loop for the last state
    return !kripke_graph_->intersecting_run(af_);
}