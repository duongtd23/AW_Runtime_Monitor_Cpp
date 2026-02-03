#ifndef ESTIMATED_KINEMATIC_HPP
#define ESTIMATED_KINEMATIC_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string ESTIMATED_KIN_TOPIC_NAME = "/api/vehicle/kinematics";
const std::string ESTIMATED_KIN_MSG_TYPE_STR = "autoware_adapi_v1_msgs/msg/VehicleKinematics";

class EstimatedKinematicTopic : public Topic
{
public:
    EstimatedKinematicTopic()
        : Topic(ESTIMATED_KIN_TOPIC_NAME, ESTIMATED_KIN_MSG_TYPE_STR) {}
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
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
            j["twist"]["angular"] = rosAngularVelToJsonPoint(kinematic_msg.twist.twist.twist.angular);

            j["acceleration"]["linear"] = vector3ToJsonPoint(kinematic_msg.accel.accel.accel.linear);
            j["acceleration"]["angular"] = rosAngularVelToJsonPoint(kinematic_msg.accel.accel.accel.angular);

        } catch (const std::exception& e) {
            // Handle deserialization errors
            j["error"] = "Failed to deserialize message: " + std::string(e.what());
            j["timestamp"] = 0.0;
        }
        
        return j;
    }

    std::string traceKey() override {
        return EstimatedKinematicTopic::TRACE_KEY();
    }
    static std::string TRACE_KEY() { return "ego_estimated_kinematic"; }
};

inline glm::vec3 getCurrentVelocity(const nlohmann::json& recorded_messages) {
    if (recorded_messages.find(EstimatedKinematicTopic::TRACE_KEY()) == recorded_messages.end() ||
        recorded_messages.at(EstimatedKinematicTopic::TRACE_KEY()).empty()) {
            return glm::vec3(0.0f);
    }
    auto kin = recorded_messages[EstimatedKinematicTopic::TRACE_KEY()].back();
    return jsonPointToVector3(kin["twist"]["linear"]);
}

inline float getCurrentSpeed(const nlohmann::json& recorded_messages) {
    return glm::length(getCurrentVelocity(recorded_messages));
}

#endif