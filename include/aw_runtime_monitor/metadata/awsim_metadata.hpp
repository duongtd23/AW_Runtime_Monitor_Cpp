#ifndef AWSIM_METADATA_HPP
#define AWSIM_METADATA_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "std_msgs/msg/string.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string AWSIM_METADATA_TOPIC_NAME = "/awsim/sim_metadata";
const std::string AWSIM_METADATA_MSG_TYPE_STR = "std_msgs/msg/String";

class AWSIMMetadata : public Topic
{
public:
    AWSIMMetadata();

    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "metadata"; }
};

#endif