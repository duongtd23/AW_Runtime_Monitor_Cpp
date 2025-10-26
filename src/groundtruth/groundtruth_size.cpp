#include <string>
#include "aw_runtime_monitor/groundtruth/groundtruth_size.hpp"
#include "nlohmann/json.hpp" // For JSON functionality
#include "rclcpp/serialization.hpp"
#include "aw_runtime_monitor/utils.hpp"

GroundtruthSizeTopic::GroundtruthSizeTopic() 
        : Topic(GROUNDTRUTH_SIZE_TOPIC_NAME, GROUNDTRUTH_SIZE_MSG_TYPE_STR, false) {
}
std::string GroundtruthSizeTopic::traceKey() {
    return GroundtruthSizeTopic::TRACE_KEY();
}

nlohmann::json GroundtruthSizeTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        aw_monitor::msg::GroundtruthSize groundtruth_size_msg;
        rclcpp::Serialization<aw_monitor::msg::GroundtruthSize> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &groundtruth_size_msg);
        
        j["camera_screen_width"] = groundtruth_size_msg.camera_screen_width;
        j["camera_screen_height"] = groundtruth_size_msg.camera_screen_height;
        j["other_note"] = groundtruth_size_msg.other_note;
        j["vehicle_sizes"] = nlohmann::json::array();

        for (auto shape : groundtruth_size_msg.vehicle_sizes) {
            nlohmann::json shape_json;
            shape_json["name"] = shape.name;
            shape_json["center"] = vector3ToJsonPoint(shape.center);
            shape_json["size"] = vector3ToJsonPoint(shape.size);
            j["vehicle_sizes"].emplace_back(shape_json);
        }
    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
    }
    return j;
}