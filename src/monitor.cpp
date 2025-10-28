#include "rclcpp/rclcpp.hpp"
#include "aw_runtime_monitor/recorder/aw_recorder.hpp"

class AWRuntimeMonitor
{
public:
    AWRuntimeMonitor(std::shared_ptr<AWRecorder> recorder) : recorder_(recorder) {
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
    topics.push_back(std::make_shared<CameraFootageTopic>());
    // for tracking autonomous driving state, e.g., starting moving, goal arrived
    topics.push_back(std::make_shared<OperationModeTrackerTopic>());
    topics.push_back(std::make_shared<RouteStateTrackerTopic>());

    auto recorder = std::make_shared<AWRecorder>(topics);
    AWRuntimeMonitor monitor(recorder);

    // Create subscriptions for all topics
    recorder->createSubscriptions();

    rclcpp::on_shutdown([recorder](){
        std::cout << "Shutting down gracefully (Ctrl+C detected)!" << std::endl;
        recorder->cliInterrupt();
    });

    // Spin the node
    rclcpp::spin(recorder);
    rclcpp::shutdown();
    
    return 0;
}