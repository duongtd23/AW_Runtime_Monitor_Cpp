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
    EstimatedKinematicTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
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