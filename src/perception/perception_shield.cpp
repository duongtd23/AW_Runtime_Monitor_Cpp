#include "aw_runtime_monitor/perception/perception_shield.hpp"
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

std::optional<autoware_perception_msgs::msg::PredictedObjects> PerceptionShield::verify(
        const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg, nlohmann::json& recorded_data) {
    if (getCurrentSpeed(recorded_data) >= this->speed_threshold_activation_) {
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

        tqtl::QualityValue result = tqtl::evaluate(perception_spec_, perp_data_stream_, 0);
        if (!result.isSatisfied()) {
            // Verification failed, return modified msg
            auto end_time =  std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            RCLCPP_WARN(logger_, "Perception shield: Spec violated (verif time: %ld microseconds).", duration.count());
            return perp_obj_msg;
        }
    }
    // If the verification succeeds, return an empty optional
    return std::nullopt;
}