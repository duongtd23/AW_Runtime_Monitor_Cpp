#ifndef BBOX_PERCEPTION_OBJECT_HPP
#define BBOX_PERCEPTION_OBJECT_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"

const std::string BBOX_PREDICTED_OBJ_TOPIC_NAME = "/perception/object_recognition/detection/rois0";
const std::string BBOX_PREDICTED_OBJ_MSG_TYPE_STR = "tier4_perception_msgs/msg/DetectedObjectsWithFeature";

class BoundingBoxPerceptionObjectTopic : public Topic
{
public:
    BoundingBoxPerceptionObjectTopic() 
        : Topic(BBOX_PREDICTED_OBJ_TOPIC_NAME, BBOX_PREDICTED_OBJ_MSG_TYPE_STR) {
    }

    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
        nlohmann::json j;
        try {
            // Deserialize the message
            tier4_perception_msgs::msg::DetectedObjectsWithFeature bbox_perp_obj_msg;
            rclcpp::Serialization<tier4_perception_msgs::msg::DetectedObjectsWithFeature> serializer;
            rclcpp::SerializedMessage extracted_serialized_msg(*msg);
            serializer.deserialize_message(&extracted_serialized_msg, &bbox_perp_obj_msg);

            // Extract timestamp from the header
            j["timestamp"] = timestamp(bbox_perp_obj_msg.header.stamp);
            j["objects"] = nlohmann::json::array();

            for (auto entry : bbox_perp_obj_msg.feature_objects) {
                nlohmann::json pobj;
                pobj["existence_prob"] = roundDouble(entry.object.existence_probability);
                pobj["classification"] = nlohmann::json::array();
                for (auto cl : entry.object.classification) {
                    nlohmann::json cl_json;
                    cl_json["label"] = cl.label;
                    cl_json["probability"] = roundDouble(cl.probability);
                    pobj["classification"].emplace_back(cl_json);
                }

                // pose, velocity, accel
                pobj["bounding_box"]["x"] = entry.feature.roi.x_offset;
                pobj["bounding_box"]["y"] = entry.feature.roi.y_offset;
                pobj["bounding_box"]["width"] = entry.feature.roi.width;
                pobj["bounding_box"]["height"] = entry.feature.roi.height;
                j["objects"].emplace_back(pobj);
            }

        } catch (const std::exception& e) {
            // Handle deserialization errors
            j["error"] = "Failed to deserialize message: " + std::string(e.what());
            j["timestamp"] = 0.0;
        }
        
        return j;
    }
    std::string traceKey() override {
        return BoundingBoxPerceptionObjectTopic::TRACE_KEY();
    }
    static std::string TRACE_KEY() { return "boundingbox_perception_objects"; }

    rclcpp::QoS qosProfile() override {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        return qos;
    }
};

#endif