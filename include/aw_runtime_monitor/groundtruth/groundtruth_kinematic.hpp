#ifndef GROUNDTRUTH_KINEMATIC_HPP
#define GROUNDTRUTH_KINEMATIC_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "aw_monitor/msg/groundtruth_kinematic.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string GROUNDTRUTH_KINEMATIC_TOPIC_NAME = "/simulation/gt/kinematic";
const std::string GROUNDTRUTH_KINEMATIC_MSG_TYPE_STR = "aw_monitor/msg/GroundtruthKinematic";

class GroundtruthKinematicTopic : public Topic 
{
public:
    GroundtruthKinematicTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "groundtruth_kinematic"; }
};

#endif