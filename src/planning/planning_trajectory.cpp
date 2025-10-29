#include <string>
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nlohmann/json.hpp" // For JSON functionality
#include "rclcpp/serialization.hpp"

nlohmann::json planningMsgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);
        
        // Extract timestamp from the header
        j["timestamp"] = timestamp(trajectory_msg.header);
        j["points"] = nlohmann::json::array();
        for (auto point : trajectory_msg.points) {
            nlohmann::json point_json;
            point_json["time_from_start"] = timestamp(point.time_from_start);
            point_json["pose"]["position"] = pointToJsonPoint(point.pose.position);
            point_json["pose"]["rotation"] = quaternionToJsonEulerAngles(point.pose.orientation);
            point_json["longitudinal_velocity"] = roundDouble(point.longitudinal_velocity_mps);
            point_json["lateral_velocity"] = roundDouble(point.lateral_velocity_mps);
            point_json["acceleration"] = roundDouble(point.acceleration_mps2);
            j["points"].emplace_back(point_json);
        }
    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
        j["timestamp"] = 0.0;
    }
    return j;
}

PlanningTrajectoryTopic::PlanningTrajectoryTopic() 
        : Topic(PLTR_TOPIC_NAME, PLTR_MSG_TYPE_STR) {
}
std::string PlanningTrajectoryTopic::traceKey() {
    return PlanningTrajectoryTopic::TRACE_KEY();
}
nlohmann::json PlanningTrajectoryTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    return planningMsgToJson(msg);
}

UnverifiedPlanningTrajectoryTopic::UnverifiedPlanningTrajectoryTopic() 
        : Topic(PLTR_UNVERIFIED_TOPIC_NAME, PLTR_MSG_TYPE_STR) {
}
std::string UnverifiedPlanningTrajectoryTopic::traceKey() {
    return UnverifiedPlanningTrajectoryTopic::TRACE_KEY();
}
nlohmann::json UnverifiedPlanningTrajectoryTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    return planningMsgToJson(msg);
}