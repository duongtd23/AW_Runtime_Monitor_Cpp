#include <string>
#include "aw_runtime_monitor/control/control_command.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"

ControlCommandTopic::ControlCommandTopic() 
        : Topic(CONTROL_COMMAND_TOPIC_NAME, CONTROL_COMMAND_MSG_TYPE_STR, false) {
}

std::string ControlCommandTopic::traceKey() {
    return ControlCommandTopic::TRACE_KEY();
}

nlohmann::json ControlCommandTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        autoware_control_msgs::msg::Control control_msg;
        rclcpp::Serialization<autoware_control_msgs::msg::Control> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &control_msg);

        j["timestamp"] = timestamp(control_msg.stamp);
        j["control_time"] = timestamp(control_msg.control_time);

        j["lateral"]["time"] = timestamp(control_msg.lateral.stamp);
        j["lateral"]["control_time"] = timestamp(control_msg.lateral.control_time);
        j["lateral"]["steering_tire_angle"] = roundDouble(control_msg.lateral.steering_tire_angle);
        j["lateral"]["steering_tire_rotation_rate"] = roundDouble(control_msg.lateral.steering_tire_rotation_rate);
        j["lateral"]["is_defined_steering_tire_rotation_rate"] = control_msg.lateral.is_defined_steering_tire_rotation_rate;

        j["longitudinal"]["time"] = timestamp(control_msg.longitudinal.stamp);
        j["longitudinal"]["control_time"] = timestamp(control_msg.longitudinal.control_time);
        j["longitudinal"]["velocity"] = roundDouble(control_msg.longitudinal.velocity);
        j["longitudinal"]["acceleration"] = roundDouble(control_msg.longitudinal.acceleration);
        j["longitudinal"]["jerk"] = roundDouble(control_msg.longitudinal.jerk);
        j["longitudinal"]["is_defined_accel"] = control_msg.longitudinal.is_defined_acceleration;
        j["longitudinal"]["is_defined_jerk"] = control_msg.longitudinal.is_defined_jerk;
    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
    }
    return j;
}

