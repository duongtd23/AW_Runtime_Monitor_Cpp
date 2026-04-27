#include "rclcpp/rclcpp.hpp"
#include "aw_runtime_monitor/recorder/aw_recorder.hpp"
#include <rclcpp/serialization.hpp>
#include <spot/misc/version.hh>
#include <cv_bridge/cv_bridge.h>
#include <utility>

// using std::placeholders::_1;
const std::string PLANNING_SPEC = "[] (pathConfidence >= 0.1 /\\ time <= 3.0 -> ~ collision)";
const std::string SPEC_SYNTAX_FILE_PATH = "formal_spec/syntax.maude";
const std::string TRACE_FILE_PATH = "recorded_data";

/**
 * @brief Load revision config
 */
RevisionConfig AWRecorder::loadRevisionConfig() {
    RevisionConfig revision_config;
    // Load deceleration profiles
    std::vector<std::string> profile_names;
    std::vector<double> target_speed_ratios, max_decel_ratios, max_jerk_ratios, weights;
    
    // Load parameters for deceleration profiles
    this->get_parameter("revision_deceleration_profiles.names", profile_names);
    this->get_parameter("revision_deceleration_profiles.target_speed_ratios", target_speed_ratios);
    this->get_parameter("revision_deceleration_profiles.max_decel_ratios", max_decel_ratios);
    this->get_parameter("revision_deceleration_profiles.max_jerk_ratios", max_jerk_ratios);
    this->get_parameter("revision_deceleration_profiles.weights", weights);
    
    // Construct deceleration profiles from loaded parameters
    revision_config.deceleration_profiles.clear();
    size_t num_profiles = std::min({profile_names.size(), target_speed_ratios.size(), 
                                    max_decel_ratios.size(), max_jerk_ratios.size(), weights.size()});
    for (size_t i = 0; i < num_profiles; ++i) {
        revision_config.deceleration_profiles.emplace_back(
            profile_names[i], target_speed_ratios[i], max_decel_ratios[i], 
            max_jerk_ratios[i], weights[i]);
    }
    
    // Load parameters for lateral shift
    this->get_parameter("revision_lateral_shift.offsets", revision_config.lateral_offsets);
    this->get_parameter("revision_lateral_shift.weight_base", revision_config.lateral_weight_base);
    this->get_parameter("revision_lateral_shift.weight_factor", revision_config.lateral_weight_factor);

    // Load parameters for combined corrections
    std::vector<std::string> combined_decel_profile_names;
    std::vector<double> combined_target_speed_ratios, combined_max_decel_ratios, combined_max_jerk_ratios, combined_weights;

    this->get_parameter("revision_combined.lateral_offsets", revision_config.combined_lateral_offsets);
    this->get_parameter("revision_combined.weight_base", revision_config.combined_weight_base);
    this->get_parameter("revision_combined.weight_factor", revision_config.combined_weight_factor);

    this->get_parameter("revision_combined.decel_profiles.names", combined_decel_profile_names);
    this->get_parameter("revision_combined.decel_profiles.target_speed_ratios", combined_target_speed_ratios);
    this->get_parameter("revision_combined.decel_profiles.max_decel_ratios", combined_max_decel_ratios);
    this->get_parameter("revision_combined.decel_profiles.max_jerk_ratios", combined_max_jerk_ratios);
    this->get_parameter("revision_combined.decel_profiles.weights", combined_weights);

    revision_config.combined_decel_profiles.clear();
    size_t num_combined_profiles = std::min({combined_decel_profile_names.size(), combined_target_speed_ratios.size(),
                                            combined_max_decel_ratios.size(), combined_max_jerk_ratios.size(), combined_weights.size()});
    for (size_t i = 0; i < num_combined_profiles; ++i) {
        revision_config.combined_decel_profiles.emplace_back(
            combined_decel_profile_names[i], combined_target_speed_ratios[i], combined_max_decel_ratios[i],
            combined_max_jerk_ratios[i], combined_weights[i]);
    }

    // Load other parameters
    this->get_parameter("revision_smoothing_iterations", revision_config.smoothing_iterations);
    this->get_parameter("revision_smoothing_weight", revision_config.smoothing_weight);
    this->get_parameter("revision_max_curvature", revision_config.max_curvature);
    this->get_parameter("revision_max_candidates", revision_config.max_candidates);
    this->get_parameter("revision_min_points", revision_config.min_points);

    // RCLCPP_INFO(this->get_logger(), "Loaded %zu deceleration profiles for trajectory revision", 
    //             revision_config.deceleration_profiles.size());
    // RCLCPP_INFO(this->get_logger(), "Loaded %zu lateral shift offsets for trajectory revision", 
    //             revision_config.lateral_offsets.size());
    // RCLCPP_INFO(this->get_logger(), "Loaded %zu combined corrections for trajectory revision", 
    //             revision_config.combined_decel_profile_indices.size());
    return revision_config;
}

/**
 * @brief Initialize parameters, e.g., output path, no_sim
 */
void AWRecorder::initializeParameters() {
    // CLI configurable parameters (which normally change per run)
    this->declare_parameter<std::string>("output_path", TRACE_FILE_PATH);
    this->declare_parameter<int>("no_sim", 1);

    // Parameters that are less likely to change per run
    this->declare_parameter<std::vector<std::string>>("topics", topic_names_);
    // this->declare_parameter<std::string>("planning_trajectory_topic", "/planning/trajectory");

    // planning shield parameters (config in default.yaml)
    this->declare_parameter<bool>("planning_monitor", false);
    this->declare_parameter<bool>("planning_shield", false);
    this->declare_parameter<std::string>("spec_syntax_file_path", SPEC_SYNTAX_FILE_PATH);
    this->declare_parameter<std::string>("planning_safety_spec", PLANNING_SPEC);
    this->declare_parameter<double>("speed_threshold_activation", 3.0);
    this->declare_parameter<double>("time_bound", 10.0);
    this->declare_parameter<double>("distance_bound", 80.0);

    // Deceleration profile parameters (replaces velocity scaling)
    this->declare_parameter<std::vector<std::string>>("revision_deceleration_profiles.names", 
        std::vector<std::string>{});
    this->declare_parameter<std::vector<double>>("revision_deceleration_profiles.target_speed_ratios", 
        std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_deceleration_profiles.max_decel_ratios", 
        std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_deceleration_profiles.max_jerk_ratios", 
        std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_deceleration_profiles.weights", 
        std::vector<double>{});
    
    // Lateral shift parameters
    this->declare_parameter<std::vector<double>>("revision_lateral_shift.offsets", std::vector<double>{});
    this->declare_parameter<double>("revision_lateral_shift.weight_base", 5.0);
    this->declare_parameter<double>("revision_lateral_shift.weight_factor", 3.0);

    // Combined correction parameters
    this->declare_parameter<std::vector<std::string>>("revision_combined.decel_profiles.names", std::vector<std::string>{});
    this->declare_parameter<std::vector<double>>("revision_combined.decel_profiles.target_speed_ratios", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_combined.decel_profiles.max_decel_ratios", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_combined.decel_profiles.max_jerk_ratios", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_combined.decel_profiles.weights", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("revision_combined.lateral_offsets", std::vector<double>{});
    this->declare_parameter<double>("revision_combined.weight_base", 10.0);
    this->declare_parameter<double>("revision_combined.weight_factor", 2.0);

    // Other revision parameters
    this->declare_parameter<int>("revision_smoothing_iterations", 5);
    this->declare_parameter<double>("revision_smoothing_weight", 0.2);
    this->declare_parameter<double>("revision_max_curvature", 0.2);
    this->declare_parameter<int>("revision_max_candidates", 50);
    this->declare_parameter<int>("revision_min_points", 30);

    // perception shield parameters
    this->declare_parameter<bool>("perception_monitor", false);
    this->declare_parameter<bool>("perception_shield", false);
    this->declare_parameter<std::string>("perception_spec", "T");
    this->declare_parameter<double>("speed_threshold_perp_shield_activation", 3.0);
    this->declare_parameter<int>("window_size", 3);
    this->declare_parameter<int>("max_prediction_frames", 3);
}

/**
 * @brief Initialize the perception shield
 */
void AWRecorder::perceptionShieldInit() {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    shielded_perception_publisher_ =
        this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(PREDICTED_OBJ_TOPIC_NAME, qos);

    if (perception_monitor_enabled_) {
        std::string perception_spec_str;
        double speed_threshold_activation;
        int max_prediction_frames;
        int window_size;
        this->get_parameter("speed_threshold_perp_shield_activation", speed_threshold_activation);
        this->get_parameter("perception_spec", perception_spec_str);
        this->get_parameter("max_prediction_frames", max_prediction_frames);
        this->get_parameter("window_size", window_size);

        this->perception_shield_ = PerceptionShield(perception_spec_str, window_size, speed_threshold_activation, max_prediction_frames, perception_shield_enabled_);

        RCLCPP_INFO(this->get_logger(), "Perception monitoring is enabled, formula: %s, window size: %d, speed threshold: %f, max prediction frames: %d", 
            perception_shield_.getSpecFormula()->toString().c_str(), window_size, speed_threshold_activation, max_prediction_frames);
    }
}

/**
 * @brief There are two possible topic names, depending on the Autoware version:
 *   - /planning/scenario_planning/trajectory
 *   - /planning/trajectory
 * This function check if the second one exists, if yes, use it; otherwise, use the first one.
 * @return 
 */
std::string AWRecorder::getPlannedTrajectoryTopicName() {
    // Check if /planning/trajectory topic exists
    auto topic_names_and_types = this->get_topic_names_and_types();
    if (topic_names_and_types.find("/planning/trajectory") != topic_names_and_types.end()) {
        return "/planning/trajectory";
    } else {
        return "/planning/scenario_planning/trajectory";
    }
}

/**
 * @brief Initialize the planning shield
 */
void AWRecorder::planningShieldInit(std::string planning_trajectory_topic_name) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    verified_trajectory_publisher_ =
        this->create_publisher<autoware_planning_msgs::msg::Trajectory>(planning_trajectory_topic_name, qos);
    verified_motion_velocity_publisher_ =
        this->create_publisher<autoware_planning_msgs::msg::Trajectory>(SCENARIO_PLTR_TOPIC_NAME, qos);

    if (planning_monitor_enabled_) {
        // If planning shield is enabled, also subscribe to scenario planning trajectory topic.
        // This is for inserting a stopping point, otherwise ego will not stop.
        auto topic = std::make_shared<ScenarioPlanningTrajectoryTopic>();
        topics_.push_back(topic);

        std::string safety_formula, spec_syntax_file_path;
        double speed_threshold_activation, time_bound, distance_bound;

        this->get_parameter("spec_syntax_file_path", spec_syntax_file_path);
        this->get_parameter("planning_safety_spec", safety_formula);
        this->get_parameter("speed_threshold_activation", speed_threshold_activation);
        this->get_parameter("time_bound", time_bound);
        this->get_parameter("distance_bound", distance_bound);

        RevisionConfig revision_config = loadRevisionConfig();
        this->planning_shield_ = PlanningShield(
            safety_formula, spec_syntax_file_path,
            speed_threshold_activation, 
            time_bound, distance_bound,
            revision_config, planning_shield_enabled_);

        RCLCPP_INFO(this->get_logger(), "Planning monitor is enabled, formula: %s", safety_formula.c_str());

        // Subscribe to map and route
        map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
            "/map/vector_map", rclcpp::QoS(1).transient_local(),
            [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
                planning_shield_.setMapForDrivableAreaChecker(msg);
                std::cout << "Map loaded" << std::endl;
            });

        route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
            "/planning/mission_planning/route", rclcpp::QoS(1).transient_local(),
            [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) {
                planning_shield_.setRouteForDrivableAreaChecker(msg);
                std::cout << "Route loaded" << std::endl;
            });


        // fetch min_acc and min_jerk from obstacle cruise planner parameters
        auto [fetched_min_acc, fetched_min_jerk] = fetchMinAccJerk();
        planning_shield_.setMinAccJerk(fetched_min_acc, fetched_min_jerk);
        
        RCLCPP_INFO(this->get_logger(), "Fetched min_acc: %.2f, min_jerk: %.2f from Autoware planner config", 
            fetched_min_acc, fetched_min_jerk);
    }
}

AWRecorder::AWRecorder() : Node("aw_recorder") {
    this->initializeParameters();

    std::string planning_trajectory_topic_name = this->getPlannedTrajectoryTopicName();
    // Load parameter values
    this->get_parameter("output_path", output_path_);
    this->get_parameter("no_sim", no_sim_);
    this->get_parameter("topics", topic_names_);
    this->get_parameter("perception_monitor", perception_monitor_enabled_);
    this->get_parameter("perception_shield", perception_shield_enabled_);
    this->get_parameter("planning_monitor", planning_monitor_enabled_);
    this->get_parameter("planning_shield", planning_shield_enabled_);


    // parse topics
    for (const auto& name : topic_names_) {
        if (name == "ControlCommand") {
            topics_.push_back(std::make_shared<ControlCommandTopic>());
        } else if (name == "GroundtruthKinematic") {
            topics_.push_back(std::make_shared<GroundtruthKinematicTopic>());
        } else if (name == "GroundtruthSize") {
            topics_.push_back(std::make_shared<GroundtruthSizeTopic>());
        } else if (name == "EstimatedKinematic") {
            topics_.push_back(std::make_shared<EstimatedKinematicTopic>());
        } else if (name == "AWSIMMetadata") {
            topics_.push_back(std::make_shared<AWSIMMetadata>());
        } else if (name == "PerceptionObject") {
            topics_.push_back(std::make_shared<PerceptionObjectTopic>(perception_monitor_enabled_));
        } else if (name == "BoundingBoxPerceptionObject") {
            topics_.push_back(std::make_shared<BoundingBoxPerceptionObjectTopic>());
        } else if (name == "PlanningTrajectory") {
            topics_.push_back(std::make_shared<PlanningTrajectoryTopic>(planning_trajectory_topic_name, planning_monitor_enabled_));
        } else if (name == "CameraFootage") {
            topics_.push_back(std::make_shared<CameraFootageTopic>());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown topic name: %s", name.c_str());
        }
    }

    // for tracking autonomous driving state, e.g., starting moving, goal arrived
    topics_.push_back(std::make_shared<OperationModeTrackerTopic>());
    topics_.push_back(std::make_shared<RouteStateTrackerTopic>());

    this->perceptionShieldInit();
    this->planningShieldInit(planning_trajectory_topic_name);

    this->reset();
}

void AWRecorder::reset() {
    this->is_recording_ = false;
    // if (planning_shield_enabled_)
    //     this->planning_shield_.verification_times_.clear();

    for (auto topic : topics_) {
        if (topic->topic_name == GROUNDTRUTH_KINEMATIC_TOPIC_NAME ||
            topic->topic_name == ESTIMATED_KIN_TOPIC_NAME ||
            dynamic_cast<PlanningTrajectoryTopic*>(topic.get()) ||
            dynamic_cast<ScenarioPlanningTrajectoryTopic*>(topic.get()) ||
            dynamic_cast<PerceptionObjectTopic*>(topic.get()) ||
            topic->topic_name == BBOX_PREDICTED_OBJ_TOPIC_NAME ||
            topic->topic_name == CONTROL_COMMAND_TOPIC_NAME) {
                if (topic->save_data)
                  this->recorded_data_[topic->traceKey()] = nlohmann::json::array();
        } 
        else if (topic->topic_name == GROUNDTRUTH_SIZE_TOPIC_NAME ||
                topic->topic_name == AWSIM_METADATA_TOPIC_NAME) {
            this->recorded_data_[topic->traceKey()].clear();
        } 
    }
    if (perception_shield_enabled_) {
        // to record the revised perception messages
        this->recorded_data_[PerceptionObjectTopic::SHIELDED_TRACE_KEY()] = nlohmann::json::array();
        this->perception_shield_.reset();
    } else if (perception_monitor_enabled_) {
        // to record the frames that violated the perception spec
        this->recorded_data_[PerceptionObjectTopic::VIOLATED_FRAMES_TRACE_KEY()] = nlohmann::json::array();
    }
    if (planning_shield_enabled_) {
        // to record the revised planned trajectories
        this->recorded_data_[PlanningTrajectoryTopic::SHIELDED_TRACE_KEY()] = nlohmann::json::array();
    } else if (planning_monitor_enabled_) {
        // to record the frames that violated the planner spec
        this->recorded_data_[PlanningTrajectoryTopic::VIOLATED_FRAMES_TRACE_KEY()] = nlohmann::json::array();
    }
    this->frames_.clear();
}

void AWRecorder::createSubscriptions() {
    for (auto topic : topics_) {
        if (topic->topic_name == CAMERA_FOOTAGE_TOPIC_NAME) {
            auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

            image_transport::TransportHints hints(this);
            camera_footage_sub_ = image_transport::create_subscription(
                this,
                CAMERA_FOOTAGE_TOPIC_NAME,
                std::bind(&AWRecorder::imageCallback, this, std::placeholders::_1),
                hints.getTransport(),
                qos.get_rmw_qos_profile());
            RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", CAMERA_FOOTAGE_TOPIC_NAME.c_str());
        }
        else {
            // Create a generic subscription using the topic's message type string
            auto subscription = this->create_generic_subscription(
                topic->topic_name,
                topic->msg_type_str,
                topic->qosProfile(),
                [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    this->unifiedCallback(msg, topic);
                }
            );
            // Store the subscription to keep it alive
            subscriptions_.push_back(subscription);
            std::cout << "Created subscription for topic '" << topic->topic_name << "' with type '" << topic->msg_type_str << "'" << std::endl;
        }
    }
}

void AWRecorder::save_data(const std::shared_ptr<Topic> topic, const std::shared_ptr<rclcpp::SerializedMessage> msg) {
    if (!is_recording_) {
        return;
    }

    // Use the topic's msgToJson method to process the message
    // This will call the correct implementation based on the actual topic type (polymorphism)
    nlohmann::json json_data = topic->msgToJson(msg);

    if (topic->topic_name == GROUNDTRUTH_SIZE_TOPIC_NAME ||
        topic->topic_name == AWSIM_METADATA_TOPIC_NAME) {
            if (topic->save_data)
                this->recorded_data_[topic->traceKey()] = json_data;
    } 
    else if (dynamic_cast<PlanningTrajectoryTopic*>(topic.get()) ||
            dynamic_cast<ScenarioPlanningTrajectoryTopic*>(topic.get()) ||dynamic_cast<PerceptionObjectTopic*>(topic.get()) ||
            topic->topic_name == GROUNDTRUTH_KINEMATIC_TOPIC_NAME ||
            topic->topic_name == ESTIMATED_KIN_TOPIC_NAME ||
            topic->topic_name == BBOX_PREDICTED_OBJ_TOPIC_NAME ||
            topic->topic_name == CONTROL_COMMAND_TOPIC_NAME) {
        if (topic->save_data)
            this->recorded_data_[topic->traceKey()].emplace_back(json_data);
    }
}

void AWRecorder::unifiedCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                const std::shared_ptr<Topic> topic) {
    this->save_data(topic, msg);

    if (PlanningTrajectoryTopic::isUnshieldedTopic(topic->topic_name)) {
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);

        if (planning_monitor_enabled_) {
            // check the safety of the planned trajectory
            auto start_time = std::chrono::high_resolution_clock::now();
            auto verif_result = this->planning_shield_.verify(trajectory_msg, this->recorded_data_);

            if (!verif_result.is_safe) {
                auto end_time =  std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                auto msg_time = timestamp(trajectory_msg.header);
                RCLCPP_WARN(this->get_logger(), "Unsafe planning trajectory [%lf] detected by the planning shield. Verif time: %ld ms", msg_time, duration);

                if (!planning_shield_enabled_)
                    this->recorded_data_[PlanningTrajectoryTopic::VIOLATED_FRAMES_TRACE_KEY()].emplace_back(msg_time);
                if (planning_shield_enabled_ && verif_result.was_revised)
                    this->recorded_data_[PlanningTrajectoryTopic::SHIELDED_TRACE_KEY()].emplace_back(
                        planningTrajMsgToJson(verif_result.revised_msg));
            }
            this->verified_trajectory_publisher_->publish(verif_result.revised_msg);
        } 

        else {
            // published the original msg
            this->verified_trajectory_publisher_->publish(trajectory_msg);
        }
    }
    else if (topic->topic_name == SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME) {
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);

        if (planning_monitor_enabled_) {
            auto verif_result = this->planning_shield_.handleScenarioPlanningTrajectory(trajectory_msg, this->recorded_data_);
            if (verif_result.has_value()) {
                this->verified_motion_velocity_publisher_->publish(verif_result.value());
            } else {
                this->verified_motion_velocity_publisher_->publish(trajectory_msg);
            }
        } 

        else {
            // publish the original message
            this->verified_motion_velocity_publisher_->publish(trajectory_msg);
        }
    }
    else if (topic->topic_name == PREDICTED_OBJ_UNSHIELDED_TOPIC_NAME) {
        autoware_perception_msgs::msg::PredictedObjects perp_obj_msg;
        rclcpp::Serialization<autoware_perception_msgs::msg::PredictedObjects> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &perp_obj_msg);

        if (perception_monitor_enabled_) {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto verif_result = this->perception_shield_.verify(perp_obj_msg, this->recorded_data_);
            
            if (!verif_result.is_safe) {
                auto end_time =  std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

                auto msg_time = timestamp(perp_obj_msg.header);
                RCLCPP_WARN(this->get_logger(), "Perception spec violated [%lf] (verif time: %.1f ms).", msg_time, duration / 1000.0);
                
                if (perception_monitor_enabled_ && !perception_shield_enabled_)
                    this->recorded_data_[PerceptionObjectTopic::VIOLATED_FRAMES_TRACE_KEY()].emplace_back(msg_time);
                // write the revised message to the recorded data
                if (perception_shield_enabled_ && verif_result.is_revised)
                    this->recorded_data_[PerceptionObjectTopic::SHIELDED_TRACE_KEY()].emplace_back(
                        PerceptionObjectTopic::perpMsgToJson(verif_result.revised_msg));
            }
            shielded_perception_publisher_->publish(verif_result.is_revised ? verif_result.revised_msg : perp_obj_msg);
            this->perception_shield_.cacheRecordedMsg(perp_obj_msg);
        } 
        else {
            shielded_perception_publisher_->publish(perp_obj_msg);
        }
    }
    else if (topic->topic_name == OP_MODE_TOPIC_NAME) {
        autoware_vehicle_msgs::msg::Engage engage_msg;
        rclcpp::Serialization<autoware_vehicle_msgs::msg::Engage> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &engage_msg);

        if (engage_msg.engage && !is_recording_) {
            this->startRecording();
        }
    }
    else if (topic->topic_name == ROUTE_STATE_TOPIC_NAME) {
        autoware_adapi_v1_msgs::msg::RouteState route_state_msg;
        rclcpp::Serialization<autoware_adapi_v1_msgs::msg::RouteState> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &route_state_msg);

        if (route_state_msg.state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED &&
            is_recording_) {
            this->stopRecording();
        }
    }
}

void AWRecorder::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (!is_recording_) {
        return;
    }
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();  // clone() to store safely
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    double t = timestamp(msg->header.stamp);
    frames_.push_back({frame, t});
}

void AWRecorder::startRecording() {
    std::cout << "Recording started." << std::endl;
    is_recording_ = true;
}

void AWRecorder::stopRecording() {
    std::cout << "Recording stopped." << std::endl;
    is_recording_ = false;
    std::string output_path = this->output_path_ + "_sim" + std::to_string(no_sim_);
    this->dumpDataToFile(output_path);
    this->saveVideo(output_path);
    // this->recorded_data_.clear();
    // this->frames_.clear();

    // if (planning_shield_enabled_) {
    //     std::cout << "Verification times (ms): ";
    //     for (const auto& time : this->planning_shield_.verification_times_) {
    //         std::cout << time << " ";
    //     }
    //     std::cout << std::endl;
    // }
    this->reset();
    this->no_sim_++;
}

void AWRecorder::dumpDataToFile(const std::string& output_path) {
    if (recorded_data_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No data recorded.");
        return;
    }
    std::ofstream file(output_path + ".json");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing.\n";
        return;
    }
    // Pretty-print (indent=2)
    file << this->recorded_data_.dump(2);
    file.close();
    std::cout << "JSON written to " << output_path << ".json\n";
}

void AWRecorder::saveVideo(const std::string& output_path) {
    if (frames_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No frames recorded, skipping video save.");
        return;
    }
    // Extract timestamps
    std::vector<double> timestamps;
    for (auto &pair : frames_)
        timestamps.push_back(pair.second);

    double duration = timestamps.back() - timestamps.front();
    double fps = (duration > 0) ? (frames_.size() / duration) : 10.0;

    RCLCPP_INFO(this->get_logger(), "Approximated FPS: %.2f", fps);

    // Get frame size
    const cv::Mat &first_frame = frames_.front().first;
    int width = first_frame.cols;
    int height = first_frame.rows;

    // Prepare video writer
    std::string video_file = output_path + "_footage.mp4";
    cv::VideoWriter writer(video_file,
                           cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                           fps,
                           cv::Size(width, height));

    if (!writer.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video file for writing: %s", video_file.c_str());
      return;
    }

    // Write all frames
    for (auto &pair : frames_) {
        writer.write(pair.first);
    }
    writer.release();

    RCLCPP_INFO(this->get_logger(), "Saved %zu frames to %s", frames_.size(), video_file.c_str());

    nlohmann::json metadata = {
        {"frame_count", frames_.size()},
        {"start_time", timestamps.front()},
        {"end_time", timestamps.back()},
        {"timestamps", timestamps},
    };
    std::ofstream meta_file(output_path + "_footage.meta.json");
    if (meta_file.is_open()) {
        meta_file << metadata.dump(2);
        meta_file.close();
        // RCLCPP_INFO(this->get_logger(), "Saved video metadata to %s", (output_path + "_footage.meta.json").c_str());
    }
}

void AWRecorder::cliInterrupt() {
    if (is_recording_) {
        this->stopRecording();
    }
}