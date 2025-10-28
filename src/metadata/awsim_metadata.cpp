#include <string>
#include "aw_runtime_monitor/metadata/awsim_metadata.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"

AWSIMMetadata::AWSIMMetadata() 
        : Topic(AWSIM_METADATA_TOPIC_NAME, AWSIM_METADATA_MSG_TYPE_STR, false) {
}

std::string AWSIMMetadata::traceKey() {
    return AWSIMMetadata::TRACE_KEY();
}

nlohmann::json AWSIMMetadata::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
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

