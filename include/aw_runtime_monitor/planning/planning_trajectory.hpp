#ifndef PLANNING_TRAJECTORY_HPP
#define PLANNING_TRAJECTORY_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include "rclcpp/serialization.hpp"
#include "nlohmann/json.hpp"

const std::string PLTR_TOPIC_NAME = "/planning/scenario_planning/trajectory";
const std::string PLTR_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory";
const std::string PLTR_UNVERIFIED_TOPIC_NAME = PLTR_TOPIC_NAME + "_unverified";

// helper functions to convert planning trajectory message to JSON
inline nlohmann::json planningTrajMsgToJson(const autoware_planning_msgs::msg::Trajectory& trajectory_msg){
    nlohmann::json j;
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
    return j;
}

inline nlohmann::json planningMsgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);
        return planningTrajMsgToJson(trajectory_msg);
    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
        j["timestamp"] = 0.0;
    }
    return j;
}

class PlanningTrajectoryTopic : public Topic 
{
public:
    PlanningTrajectoryTopic(bool shielded=false)
        : Topic(shielded ? PLTR_UNVERIFIED_TOPIC_NAME : PLTR_TOPIC_NAME, PLTR_MSG_TYPE_STR) {};
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
        nlohmann::json j;
        return planningMsgToJson(msg);
    }
    std::string traceKey() override {
        return PlanningTrajectoryTopic::TRACE_KEY();
    }
    static std::string TRACE_KEY() { return "planning_trajectory"; }
    static std::string SHIELDED_TRACE_KEY() { return TRACE_KEY() + "_shielded"; }

    rclcpp::QoS qosProfile() override {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        return qos;
    }
};


#endif