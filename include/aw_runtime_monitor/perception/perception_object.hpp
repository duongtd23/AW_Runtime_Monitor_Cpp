#ifndef PERCEPTION_OBJECT_HPP
#define PERCEPTION_OBJECT_HPP

#include "aw_runtime_monitor/topic.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"

const std::string PREDICTED_OBJ_TOPIC_NAME = "/perception/object_recognition/objects";
const std::string PREDICTED_OBJ_MSG_TYPE_STR = "autoware_perception_msgs/msg/PredictedObjects";
const std::string PREDICTED_OBJ_UNSHIELDED_TOPIC_NAME = PREDICTED_OBJ_TOPIC_NAME + "_unshielded";

const unsigned char SHAPE_BOUNDING_BOX = 0;
const unsigned char SHAPE_CYLINDER = 1;
const unsigned char SHAPE_POLYGON = 2;

class PerceptionObjectTopic : public Topic
{
public:
    PerceptionObjectTopic(bool shielded=false)
        : Topic(shielded ? PREDICTED_OBJ_UNSHIELDED_TOPIC_NAME : PREDICTED_OBJ_TOPIC_NAME, PREDICTED_OBJ_MSG_TYPE_STR) {}

    // Implement pure virtual functions from Topic
    nlohmann::json msgToJson(const std::shared_ptr<rclcpp::SerializedMessage>& msg) override {
        try {
            // Deserialize the message
            autoware_perception_msgs::msg::PredictedObjects perp_obj_msg;
            rclcpp::Serialization<autoware_perception_msgs::msg::PredictedObjects> serializer;
            rclcpp::SerializedMessage extracted_serialized_msg(*msg);
            serializer.deserialize_message(&extracted_serialized_msg, &perp_obj_msg);
            return perpMsgToJson(perp_obj_msg);
        } catch (const std::exception& e) {
            nlohmann::json j;
            // Handle deserialization errors
            j["error"] = "Failed to deserialize message: " + std::string(e.what());
            j["timestamp"] = 0.0;
            return j;
        }
    }
    std::string traceKey() override {
        return PerceptionObjectTopic::TRACE_KEY();
    }
    static std::string TRACE_KEY() { return "perception_objects"; }
    static std::string SHIELDED_TRACE_KEY() { return TRACE_KEY() + "_shielded"; }

    rclcpp::QoS qosProfile() override {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        return qos;
    }

    static nlohmann::json perpMsgToJson(
            const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg){
        nlohmann::json j;
        // Extract timestamp from the header
        j["timestamp"] = timestamp(perp_obj_msg.header.stamp);
        j["objects"] = nlohmann::json::array();

        for (auto entry : perp_obj_msg.objects) {
            nlohmann::json pobj;
            pobj["id"] = uuidstr(entry.object_id.uuid);
            pobj["existence_prob"] = roundDouble(entry.existence_probability);
            pobj["classification"] = nlohmann::json::array();
            for (auto cl : entry.classification) {
                nlohmann::json cl_json;
                cl_json["label"] = cl.label;
                cl_json["probability"] = roundDouble(cl.probability);
                pobj["classification"].emplace_back(cl_json);
            }

            // pose, velocity, accel
            pobj["pose"]["position"] = pointToJsonPoint(entry.kinematics.initial_pose_with_covariance.pose.position);
            pobj["pose"]["rotation"] = quaternionToJsonEulerAngles(entry.kinematics.initial_pose_with_covariance.pose.orientation);

            pobj["twist"]["linear"] = vector3ToJsonPoint(entry.kinematics.initial_twist_with_covariance.twist.linear);
            pobj["twist"]["angular"] = rosAngularVelToJsonPoint(entry.kinematics.initial_twist_with_covariance.twist.angular);

            pobj["acceleration"]["linear"] = vector3ToJsonPoint(entry.kinematics.initial_acceleration_with_covariance.accel.linear);
            pobj["acceleration"]["angular"] = rosAngularVelToJsonPoint(entry.kinematics.initial_acceleration_with_covariance.accel.angular);

            // shape
            if (entry.shape.type == SHAPE_BOUNDING_BOX) {
                pobj["shape"]["type"] = "box";
                pobj["shape"]["size"] = vector3ToJsonPoint(entry.shape.dimensions);
            }
            else if (entry.shape.type == SHAPE_POLYGON) {
                pobj["shape"]["type"] = "polygon";
                pobj["shape"]["footprint"] = nlohmann::json::array();
                for (auto point : entry.shape.footprint.points) {
                    pobj["shape"]["footprint"].emplace_back(pointToJsonPoint(point));
                }
            }
            else {
                std::cout << "[WARNING] Shape type `Cylinder` is not handled for perception object.\n";
                pobj["shape"]["type"] = "cylinder";
                pobj["shape"]["info"] = "Not yet handled Cylinder shape";
            }

            // predict paths
            pobj["predict_paths"] = nlohmann::json::array();
            for (auto path : entry.kinematics.predicted_paths) {
                nlohmann::json path_json;
                path_json["confidence"] = roundDouble(path.confidence);
                path_json["time_step"] = timestamp(path.time_step);
                path_json["path"] = nlohmann::json::array();
                for (auto point : path.path) {
                    nlohmann::json point_json;
                    point_json["position"] = pointToJsonPoint(point.position);
                    point_json["rotation"] = quaternionToJsonEulerAngles(point.orientation);
                    path_json["path"].emplace_back(point_json);
                }
                pobj["predict_paths"].emplace_back(path_json);
            }

            j["objects"].emplace_back(pobj);
        }
        return j;
    }
};

#endif