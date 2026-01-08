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
 * @brief Initialize parameters, e.g., output path, no_sim
 */
void AWRecorder::initializeParameters() {
    // CLI configurable parameters (which normally change per run)
    this->declare_parameter<std::string>("output_path", TRACE_FILE_PATH);
    this->declare_parameter<int>("no_sim", 1);

    // planning shield parameters
    this->declare_parameter<bool>("planning_shield_enabled", false);
    // Other parameters for the planning shield (can be set in default.yaml file), which are less likely to change per run
    this->declare_parameter<std::string>("spec_syntax_file_path", SPEC_SYNTAX_FILE_PATH);
    this->declare_parameter<std::string>("safety_formula", PLANNING_SPEC);
    this->declare_parameter<std::vector<std::string>>("topics", topic_names_);
    // inner parameters
    this->declare_parameter<double>("speed_threshold_activation", 3.0);
    this->declare_parameter<double>("min_acc", -7.0);
    this->declare_parameter<double>("min_jerk", -12.0);
    this->declare_parameter<double>("acc_start_attempt", -4.0);
    this->declare_parameter<double>("jerk_start_attempt", -3.0);
    this->declare_parameter<double>("acc_increment", 1.0);
    this->declare_parameter<double>("jerk_increment", 2.66667);
    this->declare_parameter<double>("time_bound", 5.0);
    this->declare_parameter<int>("unsafe_confirmation_frames", 2);
    this->declare_parameter<double>("distance_bound", 20.0);

    // perception shield parameters
    this->declare_parameter<bool>("perception_shield_enabled", false);
    this->declare_parameter<std::string>("perception_spec", "T");
    this->declare_parameter<double>("speed_threshold_perp_shield_activation", 3.0);
}

/**
 * @brief Initialize the perception shield
 */
void AWRecorder::perceptionShieldInit() {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    shielded_perception_publisher_ =
        this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(PREDICTED_OBJ_TOPIC_NAME, qos);

    if (perception_shield_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Perception shield is enabled.");
        std::string perception_spec_str;
        double speed_threshold_activation;
        this->get_parameter("speed_threshold_perp_shield_activation", speed_threshold_activation);
        this->get_parameter("perception_spec", perception_spec_str);
        this->perception_shield_ = PerceptionShield(perception_spec_str, speed_threshold_activation);
    }
}

/**
 * @brief Initialize the planning shield
 */
void AWRecorder::planningShieldInit() {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    verified_trajectory_publisher_ =
        this->create_publisher<autoware_planning_msgs::msg::Trajectory>(PLTR_TOPIC_NAME, qos);
    verified_motion_velocity_publisher_ =
        this->create_publisher<autoware_planning_msgs::msg::Trajectory>(SCENARIO_PLTR_TOPIC_NAME, qos);

    if (planning_shield_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Planning shield is enabled.");
        std::string safety_formula, spec_syntax_file_path;
        this->get_parameter("spec_syntax_file_path", spec_syntax_file_path);
        this->get_parameter("safety_formula", safety_formula);

        double min_acc, min_jerk, acc_start_attempt, jerk_start_attempt, acc_increment, jerk_increment;
        double speed_threshold_activation, time_bound, distance_bound;
        int unsafe_confirmation_frames;
        this->get_parameter("speed_threshold_activation", speed_threshold_activation);
        this->get_parameter("min_acc", min_acc);
        this->get_parameter("min_jerk", min_jerk);
        this->get_parameter("acc_start_attempt", acc_start_attempt);
        this->get_parameter("jerk_start_attempt", jerk_start_attempt);
        this->get_parameter("acc_increment", acc_increment);
        this->get_parameter("jerk_increment", jerk_increment);
        this->get_parameter("time_bound", time_bound);
        this->get_parameter("unsafe_confirmation_frames", unsafe_confirmation_frames);
        this->get_parameter("distance_bound", distance_bound);
        this->planning_shield_ = PlanningShield(
            verified_trajectory_publisher_,
            verified_motion_velocity_publisher_,
            safety_formula, spec_syntax_file_path,
            speed_threshold_activation, min_acc, min_jerk,
            acc_start_attempt, jerk_start_attempt, acc_increment, jerk_increment, 
            time_bound, unsafe_confirmation_frames, distance_bound);

        // if planning shield is enabled, add necessary topics to subscribe
        if (std::find(topic_names_.begin(), topic_names_.end(), "UnverifiedPlanningTrajectory") == topic_names_.end()) {
            auto topic = std::make_shared<UnverifiedPlanningTrajectoryTopic>();
            topic->save_data = false;
            topics_.push_back(topic);
        }
        if (std::find(topic_names_.begin(), topic_names_.end(), "UnverifiedScenarioPlanningTrajectory") == topic_names_.end()) {
            auto topic = std::make_shared<UnverifiedScenarioPlanningTrajectoryTopic>();
            topic->save_data = false;
            topics_.push_back(topic);
        }
    }
}

AWRecorder::AWRecorder() : Node("aw_recorder") {
    this->initializeParameters();

    // Load parameter values
    this->get_parameter("output_path", output_path_);
    this->get_parameter("no_sim", no_sim_);
    this->get_parameter("topics", topic_names_);
    this->get_parameter("perception_shield_enabled", perception_shield_enabled_);
    this->get_parameter("planning_shield_enabled", planning_shield_enabled_);

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
            topics_.push_back(std::make_shared<PerceptionObjectTopic>(perception_shield_enabled_));
        } else if (name == "BoundingBoxPerceptionObject") {
            topics_.push_back(std::make_shared<BoundingBoxPerceptionObjectTopic>());
        } else if (name == "PlanningTrajectory") {
            topics_.push_back(std::make_shared<PlanningTrajectoryTopic>());
        } else if (name == "UnverifiedPlanningTrajectory") {
            if (planning_shield_enabled_) {
                topics_.push_back(std::make_shared<UnverifiedPlanningTrajectoryTopic>());
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning shield is disabled, skip UnverifiedPlanningTrajectory topic.");
            }
        } else if (name == "UnverifiedScenarioPlanningTrajectory") {
            if (planning_shield_enabled_) {
                auto topic = std::make_shared<UnverifiedScenarioPlanningTrajectoryTopic>();
                topic->save_data = true;
                topics_.push_back(topic);
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning shield is disabled, skip UnverifiedScenarioPlanningTrajectory topic.");
            }
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
    this->planningShieldInit();

    this->reset();
}

void AWRecorder::reset() {
    this->is_recording_ = false;
    if (planning_shield_enabled_)
        this->planning_shield_.verification_times_.clear();

    for (auto topic : topics_) {
        if (topic->topic_name == GROUNDTRUTH_KINEMATIC_TOPIC_NAME ||
            topic->topic_name == ESTIMATED_KIN_TOPIC_NAME ||
            topic->topic_name == PLTR_UNVERIFIED_TOPIC_NAME ||
            topic->topic_name == PLTR_TOPIC_NAME ||
            topic->topic_name == PREDICTED_OBJ_TOPIC_NAME ||
            topic->topic_name == BBOX_PREDICTED_OBJ_TOPIC_NAME ||
            topic->topic_name == CONTROL_COMMAND_TOPIC_NAME ||
            topic->topic_name == SCENARIO_PLTR_TOPIC_NAME ||
            topic->topic_name == SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME) {
                if (topic->save_data)
                  this->recorded_data_[topic->traceKey()] = nlohmann::json::array();
        } 
        else if (topic->topic_name == GROUNDTRUTH_SIZE_TOPIC_NAME ||
                topic->topic_name == AWSIM_METADATA_TOPIC_NAME) {
            this->recorded_data_[topic->traceKey()].clear();
        } 
    }
    if (perception_shield_enabled_)
        this->recorded_data_[PerceptionObjectTopic::SHIELDED_TRACE_KEY()] = nlohmann::json::array();
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
    else if (dynamic_cast<PerceptionObjectTopic*>(topic.get()) ||
            topic->topic_name == GROUNDTRUTH_KINEMATIC_TOPIC_NAME ||
            topic->topic_name == ESTIMATED_KIN_TOPIC_NAME ||
            topic->topic_name == PLTR_TOPIC_NAME ||
            topic->topic_name == BBOX_PREDICTED_OBJ_TOPIC_NAME ||
            topic->topic_name == CONTROL_COMMAND_TOPIC_NAME ||
            topic->topic_name == SCENARIO_PLTR_TOPIC_NAME ||
            topic->topic_name == PLTR_UNVERIFIED_TOPIC_NAME ||
            topic->topic_name == SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME) {
        if (topic->save_data)
            this->recorded_data_[topic->traceKey()].emplace_back(json_data);
    }
}

void AWRecorder::unifiedCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                const std::shared_ptr<Topic> topic) {
    this->save_data(topic, msg);

    if (topic->topic_name == PLTR_UNVERIFIED_TOPIC_NAME) {
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);
        if (planning_shield_enabled_) {
            this->planning_shield_.intervene(trajectory_msg, this->recorded_data_);
        } else {
            this->verified_trajectory_publisher_->publish(trajectory_msg);
        }
    }
    else if (topic->topic_name == SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME) {
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);

        if (planning_shield_enabled_) {
            this->planning_shield_.interveneMotionVelocityMsg(trajectory_msg);
        } else {
            this->verified_motion_velocity_publisher_->publish(trajectory_msg);
        }
    }
    else if (perception_shield_enabled_ && topic->topic_name == PREDICTED_OBJ_UNSHIELDED_TOPIC_NAME) {
        RCLCPP_DEBUG(this->get_logger(), "Unshielded perception object message received.");
        autoware_perception_msgs::msg::PredictedObjects perp_obj_msg;
        rclcpp::Serialization<autoware_perception_msgs::msg::PredictedObjects> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &perp_obj_msg);

        auto msg_or_null = this->perception_shield_.verify(perp_obj_msg, this->recorded_data_);
        if (msg_or_null.has_value()) {
            // RCLCPP_WARN(this->get_logger(), "Perception shield intervened a message.");
            // revised message was NOT yet published by the shield
            shielded_perception_publisher_->publish(msg_or_null.value());
            // write the revised message to the recorded data
            this->recorded_data_[PerceptionObjectTopic::SHIELDED_TRACE_KEY()].emplace_back(
                PerceptionObjectTopic::perpMsgToJson(msg_or_null.value()));
        } else {
            // message was safe, so just publish the original message
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

    if (planning_shield_enabled_) {
        std::cout << "Verification times (ms): ";
        for (const auto& time : this->planning_shield_.verification_times_) {
            std::cout << time << " ";
        }
        std::cout << std::endl;
    }
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