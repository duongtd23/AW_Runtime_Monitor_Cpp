#ifndef CONTROL_COMMAND_HPP
#define CONTROL_COMMAND_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string CONTROL_COMMAND_TOPIC_NAME = "/control/command/control_cmd";
const std::string CONTROL_COMMAND_MSG_TYPE_STR = "autoware_control_msgs/msg/Control";

class ControlCommandTopic : public Topic 
{
public:
    ControlCommandTopic();

    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "control_cmds"; }
};

#endif