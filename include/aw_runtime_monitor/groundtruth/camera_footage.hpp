#ifndef CAMERA_FOOTAGE_HPP
#define CAMERA_FOOTAGE_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string CAMERA_FOOTAGE_TOPIC_NAME = "/sensing/camera/traffic_light/image_raw";
const std::string CAMERA_FOOTAGE_MSG_TYPE_STR = "sensor_msgs/msg/Image";

class CameraFootageTopic : public Topic 
{
public:
    CameraFootageTopic() : Topic(CAMERA_FOOTAGE_TOPIC_NAME, CAMERA_FOOTAGE_MSG_TYPE_STR) {}

    // dummy values since never used
    std::string traceKey() override { return ""; }
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>&) override {
        return nlohmann::json();
    }
};

#endif