#include <string>
#include "aw_runtime_monitor/groundtruth/groundtruth_kinematic.hpp"
#include "nlohmann/json.hpp" // For JSON functionality
#include "rclcpp/serialization.hpp"
#include "aw_runtime_monitor/utils.hpp"

GroundtruthKinematicTopic::GroundtruthKinematicTopic() 
        : Topic(GROUNDTRUTH_KINEMATIC_TOPIC_NAME, GROUNDTRUTH_KINEMATIC_MSG_TYPE_STR) {
}
std::string GroundtruthKinematicTopic::traceKey() {
    return GroundtruthKinematicTopic::TRACE_KEY();
}

nlohmann::json GroundtruthKinematicTopic::msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg){
    nlohmann::json j;
    try {
        // Deserialize the message
        aw_monitor::msg::GroundtruthKinematic groundtruth_kinematic_msg;
        rclcpp::Serialization<aw_monitor::msg::GroundtruthKinematic> serializer;
        rclcpp::SerializedMessage extracted_serialized_msg(*msg);
        serializer.deserialize_message(&extracted_serialized_msg, &groundtruth_kinematic_msg);
        
        // Extract timestamp from the header
        j["timestamp"] = timestamp(groundtruth_kinematic_msg.stamp);
        
        j["groundtruth_ego"]["pose"]["position"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.pose.position);
        j["groundtruth_ego"]["pose"]["rotation"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.pose.rotation);
        j["groundtruth_ego"]["twist"]["linear"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.twist.linear);
        j["groundtruth_ego"]["twist"]["angular"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.twist.angular);
        j["groundtruth_ego"]["acceleration"]["linear"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.accel.linear);
        j["groundtruth_ego"]["acceleration"]["angular"] = vector3ToJsonPoint(groundtruth_kinematic_msg.groundtruth_ego.accel.angular);

        j["groundtruth_vehicles"] = nlohmann::json::array();
        for (const auto& vehicle : groundtruth_kinematic_msg.groundtruth_vehicles) {
            nlohmann::json vehicle_json;
            vehicle_json["name"] = vehicle.name;
            vehicle_json["pose"]["position"] = vector3ToJsonPoint(vehicle.pose.position);
            vehicle_json["pose"]["rotation"] = vector3ToJsonPoint(vehicle.pose.rotation);
            vehicle_json["twist"]["linear"] = vector3ToJsonPoint(vehicle.twist.linear);
            vehicle_json["twist"]["angular"] = vector3ToJsonPoint(vehicle.twist.angular);
            vehicle_json["accel"] = roundDouble(vehicle.accel);
            vehicle_json["bounding_box"]["width"] = roundDouble(vehicle.bounding_box.width);
            vehicle_json["bounding_box"]["height"] = roundDouble(vehicle.bounding_box.height);
            vehicle_json["bounding_box"]["x"] = roundDouble(vehicle.bounding_box.x);
            vehicle_json["bounding_box"]["y"] = roundDouble(vehicle.bounding_box.y);
            j["groundtruth_vehicles"].emplace_back(vehicle_json);
        }
        j["groundtruth_pedestrians"] = nlohmann::json::array();
        for (const auto& pedestrian : groundtruth_kinematic_msg.groundtruth_pedestrians) {
            nlohmann::json pedestrian_json;
            pedestrian_json["name"] = pedestrian.name;
            pedestrian_json["pose"]["position"] = vector3ToJsonPoint(pedestrian.pose.position);
            pedestrian_json["pose"]["rotation"] = vector3ToJsonPoint(pedestrian.pose.rotation);
            pedestrian_json["speed"] = roundDouble(pedestrian.speed);
            pedestrian_json["bounding_box"]["width"] = roundDouble(pedestrian.bounding_box.width);
            pedestrian_json["bounding_box"]["height"] = roundDouble(pedestrian.bounding_box.height);
            pedestrian_json["bounding_box"]["x"] = roundDouble(pedestrian.bounding_box.x);
            pedestrian_json["bounding_box"]["y"] = roundDouble(pedestrian.bounding_box.y);
            j["groundtruth_pedestrians"].emplace_back(pedestrian_json);
        }
    } catch (const std::exception& e) {
        // Handle deserialization errors
        j["error"] = "Failed to deserialize message: " + std::string(e.what());
    }
    return j;
}