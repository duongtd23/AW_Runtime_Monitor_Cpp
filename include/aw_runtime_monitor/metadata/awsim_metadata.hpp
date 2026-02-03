#ifndef AWSIM_METADATA_HPP
#define AWSIM_METADATA_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "std_msgs/msg/string.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include "rclcpp/serialization.hpp"

const std::string AWSIM_METADATA_TOPIC_NAME = "/awsim/sim_metadata";
const std::string AWSIM_METADATA_MSG_TYPE_STR = "std_msgs/msg/String";

class AWSIMMetadata : public Topic
{
public:
    AWSIMMetadata()
        : Topic(AWSIM_METADATA_TOPIC_NAME, AWSIM_METADATA_MSG_TYPE_STR) {}

    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
        nlohmann::json j;
        try {
            // Deserialize the message
            std_msgs::msg::String metadata_msg;
            rclcpp::Serialization<std_msgs::msg::String> serializer;
            rclcpp::SerializedMessage extracted_serialized_msg(*msg);
            serializer.deserialize_message(&extracted_serialized_msg, &metadata_msg);


            std::string json_string = metadata_msg.data;
            //replace("'", "\"");
            std::replace(json_string.begin(), json_string.end(), '\'', '\"');
            j = nlohmann::json::parse(json_string);
        } catch (const std::exception& e) {
            // Handle deserialization errors
            j["error"] = "Failed to deserialize message: " + std::string(e.what());
        }
        return j;
    }

    std::string traceKey() override {
        return AWSIMMetadata::TRACE_KEY();
    }

    static std::string TRACE_KEY() { return "metadata"; }
};

#endif