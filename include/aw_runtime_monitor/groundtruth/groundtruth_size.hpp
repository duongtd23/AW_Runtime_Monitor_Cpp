#ifndef GROUNDTRUTH_SIZE_HPP
#define GROUNDTRUTH_SIZE_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "aw_monitor/msg/groundtruth_size.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string GROUNDTRUTH_SIZE_TOPIC_NAME = "/simulation/gt/size";
const std::string GROUNDTRUTH_SIZE_MSG_TYPE_STR = "aw_monitor/msg/GroundtruthSize";

class GroundtruthSizeTopic : public Topic 
{
public:
    GroundtruthSizeTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "groundtruth_size"; }
};

#endif