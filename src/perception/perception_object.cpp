#include <string>
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"


std::string PerceptionObjectTopic::traceKey() {
    return PerceptionObjectTopic::TRACE_KEY();
}

nlohmann::json PerceptionObjectTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    try {
        // Deserialize the message
        autoware_perception_msgs::msg::PredictedObjects perp_obj_msg;
        rclcpp::Serialization<autoware_perception_msgs::msg::PredictedObjects> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &perp_obj_msg);
        return perpMsgToJson(perp_obj_msg);
    } catch (const std::exception& e) {
        nlohmann::json j;
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
        j["timestamp"] = 0.0;
        return j;
    }
}

