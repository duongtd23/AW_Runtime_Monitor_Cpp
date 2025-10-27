#include "rclcpp/rclcpp.hpp"
#include "aw_runtime_monitor/aw_recorder.hpp"
#include <rclcpp/serialization.hpp>
#include <spot/misc/version.hh>

using std::placeholders::_1;
const std::string SAFETY_FORMULA = "[] (pathConfidence >= 0.1 /\\ time <= 3.0 -> ~ collision)";
const std::string SPEC_SYNTAX_FILE_PATH = "formal_spec/syntax.maude";

AWRecorder::AWRecorder(std::vector<std::shared_ptr<Topic>> topics) 
        : Node("aw_recorder"), topics_(topics) {
    
    this->declare_parameter<std::string>("safety_formula", SAFETY_FORMULA);
    this->declare_parameter<std::string>("spec_syntax_file_path", SPEC_SYNTAX_FILE_PATH);
    std::string safety_formula_;
    std::string spec_syntax_file_path;
    this->get_parameter("safety_formula", safety_formula_);
    this->get_parameter("spec_syntax_file_path", spec_syntax_file_path);

    this->plan_shield_ = PlanningShield(safety_formula_, spec_syntax_file_path);
    this->recorded_data_[PlanningTrajectoryTopic::TRACE_KEY()] = nlohmann::json::array();
    this->recorded_data_[EstimatedKinematicTopic::TRACE_KEY()] = nlohmann::json::array();
    this->recorded_data_[PerceptionObjectTopic::TRACE_KEY()] = nlohmann::json::array();
    // to check if spot exists
    // std::cout << "Hello world!\nThis is Spot " << spot::version() << ".\n";
}

void AWRecorder::createSubscriptions() {
    for (auto topic : topics_) {
        // Create a generic subscription using the topic's message type string
        auto subscription = this->create_generic_subscription(
            topic->topic_name,
            topic->msg_type_str,
            topic->qosProfile(),
            [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                this->unifiedCallback(msg, topic);
            }
        );
        
        // Store the subscription to keep it alive
        subscriptions_.push_back(subscription);
        
        std::cout << "Created subscription for topic '" << topic->topic_name << "' with type '" << topic->msg_type_str << "'" << std::endl;
    }
}

void AWRecorder::save_data(const std::shared_ptr<Topic> topic, const std::shared_ptr<rclcpp::SerializedMessage> msg) {
    // Use the topic's msgToJson method to process the message
    // This will call the correct implementation based on the actual topic type (polymorphism)
    nlohmann::json json_data = topic->msgToJson(msg);

    // std::cout << json_data.dump(2) << std::endl;

    if (topic->topic_name == GROUNDTRUTH_SIZE_TOPIC_NAME) {
        // if (!this->recorded_data_.contains(GroundtruthSizeTopic::TRACE_KEY())) {
        this->recorded_data_[GroundtruthSizeTopic::TRACE_KEY()] = json_data;
        // }
    } 
    else if (topic->topic_name == ESTIMATED_KIN_TOPIC_NAME) {
        this->recorded_data_[EstimatedKinematicTopic::TRACE_KEY()].emplace_back(json_data);
    } else if (topic->topic_name == PLTR_TOPIC_NAME) {
        this->recorded_data_[PlanningTrajectoryTopic::TRACE_KEY()].emplace_back(json_data);
    } else if (topic->topic_name == PREDICTED_OBJ_TOPIC_NAME) {
        this->recorded_data_[PerceptionObjectTopic::TRACE_KEY()].emplace_back(json_data);
    }
}

void AWRecorder::unifiedCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                const std::shared_ptr<Topic> topic) {
    this->save_data(topic, msg);

    if (topic->topic_name == PLTR_TOPIC_NAME) {
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::Trajectory> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &trajectory_msg);

        bool safe = this->plan_shield_.verify(trajectory_msg, this->recorded_data_);
        if (!safe) {
            RCLCPP_ERROR(this->get_logger(), "Planning trajectory violated safety constraints!");
        }
    }
}

void AWRecorder::dumpDataToFile() {
    std::ofstream file("recorded_data.json");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing.\n";
        return;
    }
    // Pretty-print (indent=2)
    file << this->recorded_data_.dump(2);
    file.close();
    std::cout << "JSON written to recorded_data.json\n";
}