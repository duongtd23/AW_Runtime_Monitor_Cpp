#ifndef AD_STATE_TRACKER_HPP
#define AD_STATE_TRACKER_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string OP_MODE_TOPIC_NAME = "/autoware/engage";
const std::string OP_MODE_MSG_TYPE_STR = "autoware_vehicle_msgs/msg/Engage";

const std::string ROUTE_STATE_TOPIC_NAME = "/api/routing/state";
const std::string ROUTE_STATE_MSG_TYPE_STR = "autoware_adapi_v1_msgs/msg/RouteState";

class OperationModeTrackerTopic : public Topic 
{
public:
    OperationModeTrackerTopic() : Topic(OP_MODE_TOPIC_NAME, OP_MODE_MSG_TYPE_STR, false) {}
    // dummy values since never used
    std::string traceKey() override { return ""; }
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>&) override {
        return nlohmann::json();
    }
};

class RouteStateTrackerTopic : public Topic 
{
public:
    RouteStateTrackerTopic() : Topic(ROUTE_STATE_TOPIC_NAME, ROUTE_STATE_MSG_TYPE_STR, false) {}
    // dummy values since never used
    std::string traceKey() override { return ""; }
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>&) override {
        return nlohmann::json();
    }
};

#endif