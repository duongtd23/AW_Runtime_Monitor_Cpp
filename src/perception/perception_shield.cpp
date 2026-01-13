#include "aw_runtime_monitor/perception/perception_shield.hpp"
#include "aw_runtime_monitor/perception/kalman_object_prediction.hpp"
#include <map>

std::map<int, tqtl::ObjectClass> class_map = {
    {0, tqtl::ObjectClass::Unknown},
    {1, tqtl::ObjectClass::Car},
    {2, tqtl::ObjectClass::Car},
    {3, tqtl::ObjectClass::Car},
    {4, tqtl::ObjectClass::Car},
    {6, tqtl::ObjectClass::Cyclist},
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
 * @brief Verify the perception message against the specification
 * 
 * If the verification fails, return a modified message with predicted states for dropped objects.
 * If the verification succeeds, return an empty optional.
 * 
 * @param perp_obj_msg The current perception message to verify
 * @param recorded_data Recorded data in JSON format (used to get current ego state, etc.)
 * @return std::optional<autoware_perception_msgs::msg::PredictedObjects> Modified message if verification fails, else std::nullopt
 */
std::optional<autoware_perception_msgs::msg::PredictedObjects> PerceptionShield::verify(
        const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg, nlohmann::json& recorded_data) {
    auto start_time = std::chrono::high_resolution_clock::now();
    tqtl::Frame frame;
    frame.setTimestamp(timestamp(perp_obj_msg.header.stamp));
    for (const auto& entry : perp_obj_msg.objects) {
        std::string id = uuidstr(entry.object_id.uuid);
        double probability = entry.existence_probability;
        tqtl::ObjectClass cls = takeMostHighProbClass(entry.classification);

        glm::vec3 position = rosPointToVector3(entry.kinematics.initial_pose_with_covariance.pose.position);
        glm::vec3 velocity = rosPointToVector3(entry.kinematics.initial_twist_with_covariance.twist.linear);

        frame.addObject(tqtl::DataObject(id, cls, probability, position, velocity));
    }
    perp_data_stream_.addFrame(frame);

    while (perp_data_stream_.getFrameCount() > window_size_) {
        perp_data_stream_.removeOldestFrame();
    }

    // only verify when the ego vehicle speed is above the threshold
    if (getCurrentSpeed(recorded_data) >= this->speed_threshold_activation_) {
        tqtl::QualityValue result = tqtl::evaluate(perception_spec_, perp_data_stream_, 0);
        if (!result.isSatisfied()) {
            // --- Verification failed, return modified msg ---
            if (recorded_perp_msgs_.empty()) {
                RCLCPP_WARN(logger_, "No available history data.");
                return perp_obj_msg;
            }
            auto last_perp_msg = recorded_perp_msgs_.back();
            auto current_ids = perp_data_stream_.getFrame(perp_data_stream_.length() - 1).getObjectIds();
            auto dropped_ids = findDroppedObjectIds(current_ids, last_perp_msg);
            
            // for debugging
            // RCLCPP_INFO(logger_, "Current object IDs:");
            // for (const auto& id : current_ids) {
            //     RCLCPP_INFO(logger_, " - %s", id.c_str());
            // }
            // RCLCPP_INFO(logger_, "Dropped object IDs:");
            // for (const auto& id : dropped_ids) {
            //     RCLCPP_INFO(logger_, " - %s", id.c_str());
            // }
            
            if (dropped_ids.empty()) {
                RCLCPP_ERROR(logger_, "No dropped objects found, but spec violated.");
                RCLCPP_WARN(logger_, "Timestamp: %f", timestamp(perp_obj_msg.header.stamp));
                std::cout << "Data stream: " << perp_data_stream_ << std::endl;
                return perp_obj_msg;
            }
            auto cloned_msg = perp_obj_msg;
            for (const auto& drop_id : dropped_ids) {
                auto history = extractObjectHistory(drop_id, recorded_perp_msgs_);
                KalmanObjectPrediction kalman_predictor;
                auto predicted_state = kalman_predictor.predictDroppedObject(
                    drop_id, history, 
                    perp_data_stream_.getFrame(perp_data_stream_.length() - 1).getTimestamp());
                cloned_msg.objects.push_back(predicted_state);
            }

            auto end_time =  std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            RCLCPP_WARN(logger_, "Perception shield: Spec violated (verif time: %ld microseconds).", duration.count());

            return cloned_msg;
        }
    }
    // If the verification succeeds, return an empty optional
    return std::nullopt;
}