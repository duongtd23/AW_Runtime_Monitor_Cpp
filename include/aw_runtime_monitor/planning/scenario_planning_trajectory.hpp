#ifndef SCENARIO_PLANNING_TRAJECTORY_HPP
#define SCENARIO_PLANNING_TRAJECTORY_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/utils.hpp"

const std::string SCENARIO_PLTR_TOPIC_NAME = "/planning/scenario_planning/scenario_selector/trajectory";
const std::string SCENARIO_PLTR_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory";
const std::string SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME = SCENARIO_PLTR_TOPIC_NAME + "_unverified";

class ScenarioPlanningTrajectoryTopic : public Topic 
{
public:
    ScenarioPlanningTrajectoryTopic(bool shielded=true) 
        : Topic(shielded ? SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME : SCENARIO_PLTR_TOPIC_NAME, SCENARIO_PLTR_MSG_TYPE_STR, false) {
    }
    
    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
        nlohmann::json j;
        return planningMsgToJson(msg);
    }
    std::string traceKey() override {
        return ScenarioPlanningTrajectoryTopic::TRACE_KEY();
    }
    static std::string TRACE_KEY() { return "scenario_planning_trajectory"; }
    static std::string SHIELDED_TRACE_KEY() { return TRACE_KEY() + "_shielded"; }
};

#endif