#ifndef TRAJECTORY_MANIPULATION_HPP
#define TRAJECTORY_MANIPULATION_HPP

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include <glm/glm.hpp>  // for vector
#include <glm/gtx/norm.hpp> 

// helper functions
glm::vec2 extractObjPosition(const autoware_planning_msgs::msg::TrajectoryPoint& entry);

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampleTrajectory(
        const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& original_points,
        size_t starting_point_id,
        float J=12,
        float min_acc=-7.0);

std::vector<double> solveRootPolynomial2(double a, double b, double c, bool filter_negative=true);

autoware_planning_msgs::msg::Trajectory injectMotionVelocityTrajectory(const autoware_planning_msgs::msg::Trajectory& trajectory, const glm::vec2& stop_point);


#endif