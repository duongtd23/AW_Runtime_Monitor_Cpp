#ifndef BBOX_PERCEPTION_OBJECT_HPP
#define BBOX_PERCEPTION_OBJECT_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string BBOX_PREDICTED_OBJ_TOPIC_NAME = "/perception/object_recognition/detection/rois0";
const std::string BBOX_PREDICTED_OBJ_MSG_TYPE_STR = "tier4_perception_msgs/msg/DetectedObjectsWithFeature";

class BoundingBoxPerceptionObjectTopic : public Topic
{
public:
    BoundingBoxPerceptionObjectTopic();

    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "boundingbox_perception_objects"; }

    rclcpp::QoS qosProfile() {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        return qos;
    }
};

#endif