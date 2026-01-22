#include "aw_runtime_monitor/topic.hpp"
#include "aw_runtime_monitor/metadata/awsim_metadata.hpp"
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "aw_runtime_monitor/perception/boundingbox_perception_object.hpp"
#include "aw_runtime_monitor/perception/perception_shield.hpp"
#include "aw_runtime_monitor/control/control_command.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "aw_runtime_monitor/planning/scenario_planning_trajectory.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_size.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_kinematic.hpp"
#include "aw_runtime_monitor/planning/planning_shield.hpp"
#include "aw_runtime_monitor/recorder/ad_state_tracker.hpp"
#include "aw_runtime_monitor/groundtruth/camera_footage.hpp"

#include "rclcpp/rclcpp.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

class AWRecorder : public rclcpp::Node
{
public:
    AWRecorder();

    // Method to create subscriptions for all topics
    void createSubscriptions();
    // when users interrupt via Ctrl+C
    void cliInterrupt();

private:
    nlohmann::json recorded_data_;
    std::vector<std::string> topic_names_;
    std::vector<std::shared_ptr<Topic>> topics_;
    std::string output_path_;
    // to store image frames and their timestamps
    std::vector<std::pair<cv::Mat, double>> frames_;
    image_transport::Subscriber camera_footage_sub_;

    bool planning_shield_enabled_ = false;
    PlanningShield planning_shield_;

    bool perception_shield_enabled_ = false;
    PerceptionShield perception_shield_;

    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;  // Store subscriptions to keep them alive
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_trajectory_publisher_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_motion_velocity_publisher_;

    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr shielded_perception_publisher_;

    bool is_recording_ = false;
    size_t no_sim_ = 1;

    // Subscriptions for map and route, used in planning shield
    rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
    rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;

    void initializeParameters();
    void perceptionShieldInit();
    void planningShieldInit();

    // Unified callback for all subscriptions
    void unifiedCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                        const std::shared_ptr<Topic> topic);
    void save_data(const std::shared_ptr<Topic> topic, const std::shared_ptr<rclcpp::SerializedMessage> msg);
    RevisionConfig loadRevisionConfig();
    
    void reset();
    void startRecording();
    void stopRecording();
    void dumpDataToFile(const std::string& output_path);
    void saveVideo(const std::string& output_path);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
};