#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "aw_runtime_monitor/planning/scenario_planning_trajectory.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_size.hpp"
#include "aw_runtime_monitor/planning/planning_shield.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/topic.hpp"
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

using std::placeholders::_1;

class AWRecorder : public rclcpp::Node
{
public:
    AWRecorder(std::vector<std::shared_ptr<Topic>> topics);

    // Method to create subscriptions for all topics
    void createSubscriptions();
    void dumpDataToFile();

private:
    PlanningShield plan_shield_;
    nlohmann::json recorded_data_;
    std::vector<std::shared_ptr<Topic>> topics_;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;  // Store subscriptions to keep them alive
    
    // Unified callback for all subscriptions
    void unifiedCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                        const std::shared_ptr<Topic> topic);
    void save_data(const std::shared_ptr<Topic> topic, const std::shared_ptr<rclcpp::SerializedMessage> msg);

};