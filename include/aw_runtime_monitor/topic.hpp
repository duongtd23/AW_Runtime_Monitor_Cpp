#ifndef TOPIC_HPP
#define TOPIC_HPP

#include <iostream>
#include <string>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp" // For JSON functionality

// Forward declaration
class AWRecorder;

// Represent a ROS topic
class Topic {
public:
    // Constructor to initialize Topic with topic_name, msg_type_str, and save_data flag
    Topic(const std::string& topic_name, const std::string& msg_type_str, bool save_data = true);

    std::string topic_name;           // Name of the topic
    std::string msg_type_str;         // Message type as string (e.g., "aw_monitor/msg/GroundtruthKinematic")
    bool save_data = true;                   // Flag indicating whether to save data

    // Pure virtual functions to be implemented by derived classes
    virtual nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) = 0;
    virtual std::string traceKey() = 0;

    rclcpp::QoS qosProfile() {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        return qos;
    }
};

#endif  // TOPIC_HPP
