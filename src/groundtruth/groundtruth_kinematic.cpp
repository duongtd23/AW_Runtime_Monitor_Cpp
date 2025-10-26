#include <string>
#include "aw_runtime_monitor/groundtruth/groundtruth_kinematic.hpp"
#include "nlohmann/json.hpp" // For JSON functionality
#include "rclcpp/serialization.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string GROUNDTRUTH_KINEMATIC_TOPIC_NAME = "/simulation/gt/kinematic";
const std::string GROUNDTRUTH_KINEMATIC_MSG_TYPE_STR = "aw_monitor/msg/GroundtruthKinematic";

GroundtruthKinematicTopic::GroundtruthKinematicTopic() 
        : Topic(GROUNDTRUTH_KINEMATIC_TOPIC_NAME, GROUNDTRUTH_KINEMATIC_MSG_TYPE_STR, false) {
}
std::string GroundtruthKinematicTopic::traceKey() {
    return "groundtruth_kinematic";
}

nlohmann::json GroundtruthKinematicTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        aw_monitor::msg::GroundtruthKinematic groundtruth_kinematic_msg;
        rclcpp::Serialization<aw_monitor::msg::GroundtruthKinematic> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &groundtruth_kinematic_msg);
        
        // Extract timestamp from the header
        j["timestamp"] = timestamp(groundtruth_kinematic_msg.stamp);
        
        j["groundtruth_ego"]["pose"]["position"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.pose.position);
        
    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
        j["timestamp"] = 0.0;
    }
    
    return j;
}