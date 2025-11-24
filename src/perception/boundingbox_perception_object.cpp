#include <string>
#include "aw_runtime_monitor/perception/boundingbox_perception_object.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"

BoundingBoxPerceptionObjectTopic::BoundingBoxPerceptionObjectTopic() 
        : Topic(BBOX_PREDICTED_OBJ_TOPIC_NAME, BBOX_PREDICTED_OBJ_MSG_TYPE_STR) {
}

std::string BoundingBoxPerceptionObjectTopic::traceKey() {
    return BoundingBoxPerceptionObjectTopic::TRACE_KEY();
}

nlohmann::json BoundingBoxPerceptionObjectTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
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

