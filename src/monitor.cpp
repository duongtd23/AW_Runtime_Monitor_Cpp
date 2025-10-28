#include "rclcpp/rclcpp.hpp"
#include "aw_runtime_monitor/aw_recorder.hpp"
#include "aw_runtime_monitor/planning/scenario_planning_trajectory.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_size.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_kinematic.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "aw_runtime_monitor/perception/perception_object.hpp"

// using std::placeholders::_1;

// const std::string PLTR_TOPIC_NAME = "/planning/scenario_planning/trajectory";
// // const std::string PLTR_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory";
// const std::string PLTR_UNVERIFIED_TOPIC_NAME = PLTR_TOPIC_NAME + "_unverified";

class AWRuntimeMonitor
{
public:
    AWRuntimeMonitor(std::shared_ptr<AWRecorder> recorder) : recorder_(recorder) {
        // sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        //   SCENARIO_PLTR_UNVERIFIED_TOPIC_NAME, 10,
        //   std::bind(&AWRuntimeMonitor::callback, this, _1));

        // pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
        //   "/planning/output_trajectory", 10);
    }
    void run() {
    }

private:
    std::shared_ptr<AWRecorder> recorder_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Create topics
    std::vector<std::shared_ptr<Topic>> topics;
    topics.push_back(std::make_shared<AWSIMMetadata>());
    topics.push_back(std::make_shared<PerceptionObjectTopic>());
    topics.push_back(std::make_shared<EstimatedKinematicTopic>());
    topics.push_back(std::make_shared<UnverifiedPlanningTrajectoryTopic>());
    topics.push_back(std::make_shared<PlanningTrajectoryTopic>());
    topics.push_back(std::make_shared<UnverifiedScenarioPlanningTrajectoryTopic>());
    topics.push_back(std::make_shared<GroundtruthSizeTopic>());
    topics.push_back(std::make_shared<GroundtruthKinematicTopic>());

    auto recorder = std::make_shared<AWRecorder>(topics);
    AWRuntimeMonitor monitor(recorder);

    // Create subscriptions for all topics
    recorder->createSubscriptions();

    rclcpp::on_shutdown([recorder](){
        // Your cleanup code here
        std::cout << "Shutting down gracefully (Ctrl+C detected)!" << std::endl;
        recorder->dumpDataToFile();
        // e.g., save logs, close files, etc.
    });

    // Spin the node
    rclcpp::spin(recorder);
    rclcpp::shutdown();
    
    return 0;
}