#ifndef PLANNING_TRAJECTORY_HPP
#define PLANNING_TRAJECTORY_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string PLTR_TOPIC_NAME = "/planning/scenario_planning/trajectory";
const std::string PLTR_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory";
const std::string PLTR_UNVERIFIED_TOPIC_NAME = PLTR_TOPIC_NAME + "_unverified";

class PlanningTrajectoryTopic : public Topic 
{
public:
    PlanningTrajectoryTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "planning_trajectory"; }
};

class UnverifiedPlanningTrajectoryTopic : public Topic 
{
public:
    UnverifiedPlanningTrajectoryTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "planning_trajectory_unverified"; }
};

nlohmann::json planningMsgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg);

#endif