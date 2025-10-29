#include <string>
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "nlohmann/json.hpp" // For JSON functionality
#include "rclcpp/serialization.hpp"

EstimatedKinematicTopic::EstimatedKinematicTopic() 
        : Topic(ESTIMATED_KIN_TOPIC_NAME, ESTIMATED_KIN_MSG_TYPE_STR) {
}

std::string EstimatedKinematicTopic::traceKey() {
    return EstimatedKinematicTopic::TRACE_KEY();
}

nlohmann::json EstimatedKinematicTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        autoware_adapi_v1_msgs::msg::VehicleKinematics kinematic_msg;
        rclcpp::Serialization<autoware_adapi_v1_msgs::msg::VehicleKinematics> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &kinematic_msg);
        
        // Extract timestamp from the header
        j["timestamp"] = timestamp(kinematic_msg.geographic_pose.header.stamp);
        j["pose"]["position"] = pointToJsonPoint(kinematic_msg.pose.pose.pose.position);
        j["pose"]["rotation"] = quaternionToJsonEulerAngles(kinematic_msg.pose.pose.pose.orientation);

        j["twist"]["linear"] = vector3ToJsonPoint(kinematic_msg.twist.twist.twist.linear);
        j["twist"]["angular"] = vector3ToJsonPoint(kinematic_msg.twist.twist.twist.angular);

        j["acceleration"]["linear"] = vector3ToJsonPoint(kinematic_msg.accel.accel.accel.linear);
        j["acceleration"]["angular"] = vector3ToJsonPoint(kinematic_msg.accel.accel.accel.angular);

    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
        j["timestamp"] = 0.0;
    }
    
    return j;
}