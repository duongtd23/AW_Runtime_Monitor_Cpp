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
    GroundtruthSizeTopic() : Topic(GROUNDTRUTH_SIZE_TOPIC_NAME, GROUNDTRUTH_SIZE_MSG_TYPE_STR) {}
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
        nlohmann::json j;
        try {
            // Deserialize the message
            aw_monitor::msg::GroundtruthSize groundtruth_size_msg;
            rclcpp::Serialization<aw_monitor::msg::GroundtruthSize> serializer;
            rclcpp::SerializedMessage extracted_serialized_msg(*msg);
            serializer.deserialize_message(&extracted_serialized_msg, &groundtruth_size_msg);
            
            j["camera_screen_width"] = groundtruth_size_msg.camera_screen_width;
            j["camera_screen_height"] = groundtruth_size_msg.camera_screen_height;
            j["other_note"] = groundtruth_size_msg.other_note;
            j["vehicle_sizes"] = nlohmann::json::array();

            for (auto shape : groundtruth_size_msg.vehicle_sizes) {
                nlohmann::json shape_json;
                shape_json["name"] = shape.name;
                shape_json["center"] = vector3ToJsonPoint(shape.center);
                shape_json["size"] = vector3ToJsonPoint(shape.size);
                j["vehicle_sizes"].emplace_back(shape_json);
            }
        } catch (const std::exception& e) {
            // Handle deserialization errors
            j["error"] = "Failed to deserialize message: " + std::string(e.what());
        }
        return j;
    }

    std::string traceKey() override {
        return GroundtruthSizeTopic::TRACE_KEY();
    }
    static std::string TRACE_KEY() { return "groundtruth_size"; }
};

#endif