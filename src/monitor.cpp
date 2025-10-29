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

    auto recorder = std::make_shared<AWRecorder>();
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