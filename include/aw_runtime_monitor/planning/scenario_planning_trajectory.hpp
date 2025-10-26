#ifndef SCENARIO_PLANNING_TRAJECTORY_HPP
#define SCENARIO_PLANNING_TRAJECTORY_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string SCENARIO_PLTR_TOPIC_NAME = "/planning/scenario_planning/scenario_selector/trajectory";
const std::string SCENARIO_PLTR_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory";
const std::string SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME = SCENARIO_PLTR_TOPIC_NAME + "_unverified";

class ScenarioPlanningTrajectoryTopic : public Topic 
{
public:
    ScenarioPlanningTrajectoryTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "scenario_planning_trajectory"; }
};

class UnverifiedScenarioPlanningTrajectoryTopic : public Topic 
{
public:
    UnverifiedScenarioPlanningTrajectoryTopic();
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override;
    std::string traceKey() override;
    static std::string TRACE_KEY() { return "scenario_planning_trajectory_unverified"; }
};

#endif