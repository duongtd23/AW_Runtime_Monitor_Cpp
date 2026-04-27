#include "aw_runtime_monitor/perception/perception_shield.hpp"
#include "aw_runtime_monitor/perception/kalman_object_prediction.hpp"
#include <map>

std::map<int, tqtl::ObjectClass> class_map = {
    {0, tqtl::ObjectClass::Unknown},
    {1, tqtl::ObjectClass::Car},
    {2, tqtl::ObjectClass::Car},
    {3, tqtl::ObjectClass::Car},
    {4, tqtl::ObjectClass::Car},
    {6, tqtl::ObjectClass::Bicycle},
    {7, tqtl::ObjectClass::Pedestrian}
};

/**
 * @brief Take the object class with the highest probability from the classification record
 * @param classification_record The classification record from autoware_perception_msgs::msg::ObjectClassification
 * @return tqtl::ObjectClass The object class with the highest probability
 */
tqtl::ObjectClass takeMostHighProbClass(
        const std::vector<autoware_perception_msgs::msg::ObjectClassification>& classification_record) {
    double max_prob = -1.0;
    tqtl::ObjectClass best_class = tqtl::ObjectClass::Unknown;
    for (const auto& entry : classification_record) {
        double prob = entry.probability;
        if (prob > max_prob) {
            max_prob = prob;
            int class_code = entry.label;
            best_class = class_map[class_code];
        }
    }
    return best_class;
}

/**
 * @brief Extract object history from recorded data (in JSON format)
 * @param object_id The ID of the object to extract history for
 * @param recorded_perp_msgs Recorded perception msgs in JSON array (not include the current msg)
 * @param desired_history_length The desired length of history to extract
 */
std::vector<KalmanObjectPrediction::ObjectHistoryState> extractObjectHistory(
        const std::string& object_id,
        const std::vector<autoware_perception_msgs::msg::PredictedObjects>& recorded_perp_msgs, 
        size_t desired_history_length=5) {
    std::vector<KalmanObjectPrediction::ObjectHistoryState> history;
    int i = recorded_perp_msgs.size() - 1;
    while (i >= 0 && history.size() < desired_history_length) {
        double timest = timestamp(recorded_perp_msgs[i].header.stamp);
        for (const auto& item : recorded_perp_msgs[i].objects) {
            std::string obj_id = uuidstr(item.object_id.uuid);
            if (obj_id == object_id) {
                KalmanObjectPrediction::ObjectHistoryState state;
                state.timestamp = timest;
                state.raw_object = item;
                history.push_back(state);
            }
        }
        i--;
    }
    return history;
}

/**
 * @brief Find dropped object IDs by comparing current and last recorded perception msgs
 * @param current_ids The set of current object IDs 
 * @param last_perp_msg The last recorded perception message
 */
std::vector<std::string> findDroppedObjectIds(
        const std::unordered_set<std::string>& current_ids,
        const autoware_perception_msgs::msg::PredictedObjects& last_perp_msg) {
    std::vector<std::string> dropped_ids;
    for (const auto& item : last_perp_msg.objects) {
        std::string obj_id = uuidstr(item.object_id.uuid);
        if (current_ids.find(obj_id) == current_ids.end()) {
            // Object ID not found in current message, it is dropped
            dropped_ids.push_back(obj_id);
        }
    }
    return dropped_ids;
}

/**
 * @brief Convert ROS object to tqtl DataObject
 */
tqtl::DataObject rosObjectToDataObject(
        const autoware_perception_msgs::msg::PredictedObject& ros_obj) {
    std::string id = uuidstr(ros_obj.object_id.uuid);
    double probability = ros_obj.existence_probability;
    tqtl::ObjectClass cls = takeMostHighProbClass(ros_obj.classification);

    glm::vec3 position = rosPointToVector3(ros_obj.kinematics.initial_pose_with_covariance.pose.position);
    glm::vec3 velocity = rosPointToVector3(ros_obj.kinematics.initial_twist_with_covariance.twist.linear);

    return tqtl::DataObject(id, cls, probability, position, velocity);
}

/**
 * @brief Predict the dropped object state from previous detection
 */
autoware_perception_msgs::msg::PredictedObject predictDroppedObjectFromPreviousDetection(
        const PerceptionShield::DroppedObjectPrediction& previous_detection,
        double curr_timest) {
    double last_timest = previous_detection.timestamp;
    double delta_t = curr_timest - last_timest;
    double vel_x = previous_detection.predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x;
    double vel_y = previous_detection.predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y;
    double vel_z = previous_detection.predicted_object.kinematics.initial_twist_with_covariance.twist.linear.z;
    // Use the velocity to update the position
    auto predicted_obj = previous_detection.predicted_object;
    predicted_obj.kinematics.initial_pose_with_covariance.pose.position.x += vel_x * delta_t;
    predicted_obj.kinematics.initial_pose_with_covariance.pose.position.y += vel_y * delta_t;
    predicted_obj.kinematics.initial_pose_with_covariance.pose.position.z += vel_z * delta_t;
    return predicted_obj;
}

/**
 * @brief Extract the latest ego state from the vehicle estimated kinematic in recorded data
 */
tqtl::EgoObject extractEgoLatestState(const nlohmann::json& recorded_data) {
    if (!recorded_data.contains(EstimatedKinematicTopic::TRACE_KEY()) || 
        recorded_data[EstimatedKinematicTopic::TRACE_KEY()].empty()) {
            return tqtl::EgoObject();
    }
    const auto& ego_states = recorded_data[EstimatedKinematicTopic::TRACE_KEY()];
    const auto& latest_ego = ego_states.back();
    glm::vec3 position(
        latest_ego["pose"]["position"]["x"].get<double>(),
        latest_ego["pose"]["position"]["y"].get<double>(),
        latest_ego["pose"]["position"]["z"].get<double>());
    glm::vec3 velocity(
        latest_ego["twist"]["linear"]["x"].get<double>(),
        latest_ego["twist"]["linear"]["y"].get<double>(),
        latest_ego["twist"]["linear"]["z"].get<double>());
    glm::vec3 euler_angles(
        latest_ego["pose"]["rotation"]["x"].get<double>(),
        latest_ego["pose"]["rotation"]["y"].get<double>(),
        latest_ego["pose"]["rotation"]["z"].get<double>());
    return tqtl::EgoObject(position, velocity, euler_angles);
}

/**
 * @brief Verify the perception message against the specification
 * 
 * If the verification fails, return a modified message with predicted states for dropped objects.
 * If the verification succeeds, return an empty optional.
 * 
 * @param perp_obj_msg The current perception message to verify
 * @param recorded_data Recorded data in JSON format (used to get current ego state, etc.)
 * @return VerificationResult
 */
PerceptionShield::VerificationResult PerceptionShield::verify(
        const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg, 
        const nlohmann::json& recorded_data) {
    tqtl::Frame frame;
    const double curr_timest = timestamp(perp_obj_msg.header.stamp);
    frame.setTimestamp(curr_timest);
    for (const auto& entry : perp_obj_msg.objects) {
        frame.addObject(rosObjectToDataObject(entry));
    }
    frame.setEgoObject(extractEgoLatestState(recorded_data));
    perp_data_stream_.addFrame(frame);

    while (perp_data_stream_.getFrameCount() > window_size_) {
        perp_data_stream_.removeOldestFrame();
    }

    // if the ego vehicle speed is below the threshold, skip the verification
    if (getCurrentSpeed(recorded_data) < this->speed_threshold_activation_) {
        PerceptionShield::VerificationResult verif_result;
        verif_result.is_safe = true;
        verif_result.is_revised = false;
        return verif_result;
    }
    
    // --- Ego speed is above threshold, perform verification ---
    // Evaluate the perception specification
    tqtl::QualityValue result = tqtl::evaluate(perception_spec_, perp_data_stream_, 0);

    // If revision is disabled, return immediately
    if (!enable_revision_) {
        PerceptionShield::VerificationResult verif_result;
        verif_result.is_safe = result.isSatisfied();
        verif_result.is_revised = false;
        return verif_result;
    }

    // --- Revision enabled, try to revise the message if verification fails ---
    // auto cloned_msg = perp_obj_msg;
    PerceptionShield::VerificationResult verif_result;
    verif_result.is_safe = result.isSatisfied();
    verif_result.is_revised = false;
    verif_result.revised_msg = perp_obj_msg;

    if (enable_revision_) {
        // continue predicting previously dropped objects
        for (auto it = dropped_object_predictions_.begin();
                it != dropped_object_predictions_.end(); ) {
            auto& previous_detection = *it;

            if(!frame.hasObject(uuidstr(previous_detection.predicted_object.object_id.uuid))) {
                // predict based on previously predicted velocity and pose
                auto predicted_obj = predictDroppedObjectFromPreviousDetection(
                    previous_detection, curr_timest);
                verif_result.revised_msg.objects.push_back(predicted_obj);
                verif_result.is_revised = true;
            }

            // update the dropped object prediction record
            previous_detection.no_frames_fixed += 1;

            // check if we have exceeded the max prediction frames
            if (previous_detection.no_frames_fixed >= max_prediction_frames_) {
                it = dropped_object_predictions_.erase(it);
            } else {
                ++it;
            }
        }
    }

    if (!result.isSatisfied()) {
        // --- Verification failed, return modified msg ---
        if (recorded_perp_msgs_.empty()) {
            RCLCPP_WARN(logger_, "No available history data.");
            return verif_result;
        }
        auto last_perp_msg = recorded_perp_msgs_.back();
        auto current_ids = perp_data_stream_.getFrame(perp_data_stream_.length() - 1).getObjectIds();
        auto dropped_ids = findDroppedObjectIds(current_ids, last_perp_msg);
        
        if (dropped_ids.empty()) {
            RCLCPP_ERROR(logger_, "No dropped objects found, but spec violated.");
            RCLCPP_WARN(logger_, "Timestamp: %f", curr_timest);
            std::cout << "Data stream: " << perp_data_stream_ << std::endl;
            return verif_result;
        }
        for (const auto& drop_id : dropped_ids) {
            // predict the dropped object state
            auto history = extractObjectHistory(drop_id, recorded_perp_msgs_);
            KalmanObjectPrediction kalman_predictor;
            auto predicted_state = kalman_predictor.predictDroppedObject(
                drop_id, history, 
                perp_data_stream_.getFrame(perp_data_stream_.length() - 1).getTimestamp());
            
            // confirm if adding this predicted object will help satisfy the spec
            auto data_obj = rosObjectToDataObject(predicted_state);
            auto test_stream = perp_data_stream_.cloneAndAppendObject(data_obj);
            auto test_result = tqtl::evaluate(perception_spec_, test_stream, 0);
            if (test_result.isSatisfied()) {
                verif_result.is_revised = true;
                verif_result.revised_msg.objects.push_back(predicted_state);

                // record this dropped object prediction
                DroppedObjectPrediction drop_record;
                drop_record.timestamp = curr_timest;
                drop_record.predicted_object = predicted_state;
                drop_record.no_frames_fixed = 1;
                dropped_object_predictions_.push_back(drop_record);
            }
        }
    }
    return verif_result;
}