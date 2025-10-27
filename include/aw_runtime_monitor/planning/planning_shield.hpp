#ifndef PLANNING_SHIELD_HPP
#define PLANNING_SHIELD_HPP

#include "aw_runtime_monitor/planning/trajectory_manipulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"
#include <tuple>
#include <spot/tl/parse.hh>
#include <algorithm> // Required for std::replace
#include <functional>  // for std::function
#include <map>

const float TIME_BOUND = 4.0;

class ConstantHeadingVehicle
{
public:
    ConstantHeadingVehicle(const nlohmann::json& vehicle_current_pose,
        double veh_length, double veh_width, double veh_center_x, double veh_center_y);
    std::vector<glm::vec2> getVerticesWhenAtPosition(const glm::vec2& position) const;
    std::vector<glm::vec2> getVerticesAtTime(float time, float speed);

    glm::vec2 init_pos;
    double init_rot;
    std::vector<glm::vec2> init_vertices;

private:
    glm::vec2 norm_vel_;
};

class PlanningPoint
{
public:
    glm::vec2 position;
    float velocity;
    float accel;
    float time_to_next_point; 
    float time_from_start;
    PlanningPoint(const glm::vec2& position, float velocity, float accel, 
            float time_to_next_point, float time_from_start);
};

class PlanningTrajectory
{
public:
    PlanningTrajectory(const ConstantHeadingVehicle& ego_current_state);
    PlanningTrajectory(const ConstantHeadingVehicle& ego_current_state, 
            // const std::vector<PlanningPoint>& past_points, 
            size_t start_point_id,
            const std::vector<PlanningPoint>& future_points);

    ConstantHeadingVehicle ego_current_state;
    // std::vector<PlanningPoint> past_points;
    size_t start_point_id;
    std::vector<PlanningPoint> future_points;
    
    void parseFuturePoints(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points_raw);
    void parseFuturePointsWithoutTimesteps(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points_raw, 
        size_t start_id, bool skip_tails=true);
    void parsePastFuturePoints(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& raw_points);

private:
};

using ComparisonFunction = std::function<bool(float)>;

class PlanningShield 
{
public:
    PlanningShield(float speed_threshold_activation=3.0, float min_acc=-7.0, float min_jerk=-12.0)
    : speed_threshold_activation_(speed_threshold_activation), min_acc_(min_acc), min_jerk_(min_jerk) {}

    PlanningShield(
        rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_trajectory_publisher,
        rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_motion_velocity_publisher,
        std::string spec_formula_str, std::string spec_syntax_file_path,
        float speed_threshold_activation=3.0, float min_acc=-7.0, float min_jerk=-12.0);
    
    void intervene(const autoware_planning_msgs::msg::Trajectory& trajectory_msg, 
                   const nlohmann::json& recorded_data);
    void interveneMotionVelocityMsg(const autoware_planning_msgs::msg::Trajectory& trajectory_msg);

    bool verify(const PlanningTrajectory& planning_points, 
                const nlohmann::json& perception_msgs);
    bool verifySimulatedNpcPath(const PlanningTrajectory& planning_points,
                                float existence_prob, const glm::vec2& current_position, float current_heading,
                                const glm::vec2& current_vel, const std::vector<glm::vec2>& npc_local_vertices,
                                float time_bound=TIME_BOUND);

    bool verifyPredictNpcPath(const PlanningTrajectory& planning_points,
                              float existence_prob, const nlohmann::json& predicted_path, float current_heading, const std::vector<glm::vec2>& npc_local_vertices,
                              float time_bound=TIME_BOUND);

    bool evaluateSpec(const std::vector<float>& time_series,
        const std::vector<bool>& collision_series,
        const std::vector<float>& distance_series,
        float existence_prob,
        float path_confidence);
    
    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampleTrajectoryPoints(
        const autoware_planning_msgs::msg::Trajectory& trajectory_msg, 
        size_t _id, 
        const nlohmann::json& perception_msgs, 
        const ConstantHeadingVehicle& ego_current_state);
    
    void publishMsg(const autoware_planning_msgs::msg::Trajectory& msg);

    PlanningTrajectory toPlanningTrajectoryObj(const autoware_planning_msgs::msg::Trajectory& trajectory_msg, 
        const nlohmann::json& estimated_kinematic, const nlohmann::json& ego_shape);

private:
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_trajectory_publisher_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr verified_motion_velocity_publisher_;
    float speed_threshold_activation_;
    float min_acc_;
    float min_jerk_;
    double last_stop_time_;
    glm::vec2 stop_point_;
    bool has_stop_point_ = false;
    nlohmann::json ego_shape_; // cached ego shape
    std::vector<std::string> propositions_;
    spot::parsed_formula spec_formula_;
    std::map<std::string, ComparisonFunction> proposition_map_;
    std::string spec_syntax_file_path_;
};

#endif