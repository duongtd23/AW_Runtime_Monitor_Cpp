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

#endif