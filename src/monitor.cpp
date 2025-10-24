#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <string>

using std::placeholders::_1;

std::string PLTR_TOPIC_NAME = "/planning/scenario_planning/trajectory";
std::string PLTR_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory";
std::string PLTR_UNVERIFIED_TOPIC_NAME = PLTR_TOPIC_NAME + "_unverified";

std::string SCENARIO_PLTR_TOPIC_NAME = "/planning/scenario_planning/scenario_selector/trajectory";
std::string SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME = SCENARIO_PLTR_TOPIC_NAME + "_unverified";

class AWRuntimeMonitor : public rclcpp::Node
{
public:
    AWRuntimeMonitor() : Node("aw_runtime_monitor")
    {
        sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
          SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME, 10,
          std::bind(&AWRuntimeMonitor::callback, this, _1));

        // pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
        //   "/planning/output_trajectory", 10);
    }

private:
    void callback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received %d", msg->header.stamp.sec);
        // Example: just republish empty trajectory
        // autoware_planning_msgs::msg::Trajectory traj;
        // pub_->publish(traj);
    }

    rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_;
    // rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AWRuntimeMonitor>());
    rclcpp::shutdown();
    return 0;
}