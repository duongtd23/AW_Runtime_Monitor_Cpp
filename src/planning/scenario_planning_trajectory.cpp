#include <string>
#include "aw_runtime_monitor/planning/scenario_planning_trajectory.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nlohmann/json.hpp" // For JSON functionality
#include "rclcpp/serialization.hpp"

ScenarioPlanningTrajectoryTopic::ScenarioPlanningTrajectoryTopic() 
        : Topic(SCENARIO_PLTR_TOPIC_NAME, SCENARIO_PLTR_MSG_TYPE_STR, false) {
}
std::string ScenarioPlanningTrajectoryTopic::traceKey() {
    return ScenarioPlanningTrajectoryTopic::TRACE_KEY();
}
nlohmann::json ScenarioPlanningTrajectoryTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    return planningMsgToJson(msg);
}


UnverifiedScenarioPlanningTrajectoryTopic::UnverifiedScenarioPlanningTrajectoryTopic() 
        : Topic(SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME, SCENARIO_PLTR_MSG_TYPE_STR, false) {
}

std::string UnverifiedScenarioPlanningTrajectoryTopic::traceKey() {
    return UnverifiedScenarioPlanningTrajectoryTopic::TRACE_KEY();
}
nlohmann::json UnverifiedScenarioPlanningTrajectoryTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    return planningMsgToJson(msg);
}