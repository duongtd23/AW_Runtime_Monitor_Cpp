#include "aw_runtime_monitor/topic.hpp"
#include "nlohmann/json.hpp"

// Represent a ROS topic
Topic::Topic(const std::string& topicname, const std::string& msg_type_str, bool savedata)
        : topic_name(topicname), msg_type_str(msg_type_str), save_data(savedata) {
}