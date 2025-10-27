#include "aw_runtime_monitor/planning/trajectory_manipulation.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
// for polynomial solving
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>

glm::vec2 extractObjPosition(const autoware_planning_msgs::msg::TrajectoryPoint& entry) {
    return glm::vec2(entry.pose.position.x, entry.pose.position.y);
}

glm::vec2 resamplePoint(const glm::vec2& original_point, const glm::vec2& prev_point, float desired_distance) {
    /**
    update the path point (original_point) to match the desired distance
    (which is less than the distance from prev_point to original point)
    */
    float original_dis = glm::length(original_point - prev_point);
    return original_point + desired_distance / original_dis * (original_point - prev_point);
}

// the order is reversed.
// polynomial: c*x^2 + b*x + a = 0
std::vector<double> solveRootPolynomial2(double a, double b, double c, bool filter_negative) {
    Eigen::Vector3d coeffs; // degree 2 → 3 coefficients
    coeffs << a, b, c;
    Eigen::PolynomialSolver<double, 2> solver;
    solver.compute(coeffs);
    auto roots = solver.roots();
    std::vector<double> real_roots;
    for (int i = 0; i < roots.size(); ++i) {
        if (std::abs(roots[i].imag()) < 1e-6) {
            double re = roots[i].real();
            if (filter_negative) {
                if (re >= 0) {
                    real_roots.push_back(re);
                }
            } else
                real_roots.push_back(re);
        }
    }
    return real_roots;
}

// polynomial: d*x^3 + c*x^2 + b*x + a = 0
std::vector<double> solveRootPolynomial3(double a, double b, double c, double d, bool filter_negative=true) {
    Eigen::Vector4d coeffs;
    coeffs << a, b, c, d;
    Eigen::PolynomialSolver<double, 3> solver;
    solver.compute(coeffs);
    auto roots = solver.roots();
    std::vector<double> real_roots;
    for (int i = 0; i < roots.size(); ++i) {
        if (std::abs(roots[i].imag()) < 1e-6) {
            double re = roots[i].real();
            if (filter_negative) {
                if (re >= 0) {
                    real_roots.push_back(re);
                }
            } else
                real_roots.push_back(re);
        }
    }
    return real_roots;
}

float a(float t, float J, float a0) {
    return a0 - J*t;
}
float v(float t, float J, float v0, float a0) {
    if (J == 0) {
        // constant acceleration
        float re = v0 + a0 * t;
        if (re < 0)
            return 0.0;
        return re;
    }
    return v0 + a0*t - 0.5*J*t*t;
}
float distanceTravel(float t, float J, float v0, float a0) {
    return v0*t + a0*t*t/2 - J*t*t*t/6;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampleTrajectory(
        const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& original_points,
        size_t starting_point_id, float J, float min_acc) {

    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampled_points{
        original_points[starting_point_id]
    };

    size_t i = starting_point_id + 1;
    while(i < original_points.size() - 2) {
        const auto& prev_entry = original_points[i - 1];
        glm::vec2 point0 = extractObjPosition(prev_entry);
        float v0 = resampled_points.back().longitudinal_velocity_mps;
        float a0 = resampled_points.back().acceleration_mps2;
        if (std::abs(v0) < 1e-6) {
            break;
        }

        const auto& entry = original_points[i];
        glm::vec2 point1 = extractObjPosition(entry);
        float dis = glm::length(point0 - point1);
        auto new_point = entry;
        float a1,v1,time_gap;
        if (std::abs(a0 - min_acc) < 1e-6) {
            // Case 1. Already reached max deceleration.  jerk = 0
            a1 = a0;
            // check if the stopping distance is less than distance to this planning point ($dis)
            float stopping_distance = -0.5 * v0 * v0 / a0;
            if (stopping_distance < dis) {
                // Case 1.1. stopping distance is less than distance to this planning point
                a1 = 0.0;
                v1 = 0.0;
                time_gap = -v0/a0;
                // In this case, real travel distance < dis
                glm::vec2 updated_point1 = resamplePoint(point1, point0, stopping_distance);
                new_point.pose.position.x = updated_point1[0];
                new_point.pose.position.y = updated_point1[1];
            } else {
                // Case 1.2. distance to this planning point is less than the stopping distance
                // Solve d(t) = v0*t + 0.5*a0*t^2 = dis to derive the time gap (travel time)
                auto ts = solveRootPolynomial2(-dis, v0, 0.5*a0);
                if (ts.empty()) {
                    RCLCPP_ERROR(rclcpp::get_logger("TrajectoryManipulation"), "[ERROR] Cannot determine the time gap");
                    break;
                }
                time_gap = *std::min_element(ts.begin(), ts.end());
                v1 = v(time_gap, 0, v0, a0);
            }
        } else {
            // Case 2. Not reach max deceleration.  jerk = J
            float time_to_maxbrake = (a0 - min_acc)/J;
            float v_at_maxbrake = v0 + a0*time_to_maxbrake - 0.5*J*time_to_maxbrake*time_to_maxbrake;
            if (v_at_maxbrake < 0) {
                // Case 2.1. Stop before reaching max deceleration
                // Solve v(t) = v0 + a0*t - 0.5*J*t^2 = 0 to derive the stopping time
                auto ts = solveRootPolynomial2(v0, a0, -0.5 * J);
                if (ts.empty()) {
                    RCLCPP_ERROR(rclcpp::get_logger("TrajectoryManipulation"), "[ERROR] Cannot determine the time gap");
                    break;
                }
                float stopping_time = *std::min_element(ts.begin(), ts.end());
                float stopping_distance = distanceTravel(stopping_time, J, v0, a0);
                if (stopping_distance < dis) {
                    // Case 2.1.1. Fully stop before reaching the path point
                    a1 = 0.0;
                    v1 = 0.0;
                    time_gap = stopping_time;
                    // In this case, real travel distance < dis
                    glm::vec2 updated_point1 = resamplePoint(point1, point0, stopping_distance);
                    new_point.pose.position.x = updated_point1[0];
                    new_point.pose.position.y = updated_point1[1];
                } else {
                    // Case 2.1.2. Not stop when reaching the path point
                    // Solve d(t) = v0*t + a0*t^2/2 -J*t^3/6 = dis to derive the time gap (travel time)
                    auto ts2 = solveRootPolynomial3(-dis, v0, a0/2, -J/6);
                    if (ts2.empty()) {
                        RCLCPP_ERROR(rclcpp::get_logger("TrajectoryManipulation"), "[ERROR] Cannot determine the time gap");
                        break;
                    }
                    time_gap = *std::min_element(ts2.begin(), ts2.end());
                    a1 = a(time_gap, J, a0);
                    v1 = v(time_gap, J, v0, a0);
                }
            } else {
                // Case 2.2. Does not fully stop when reaching max deceleration
                float dis_to_maxbrake = distanceTravel(time_to_maxbrake, J, v0, a0);
                float stopping_time = -v_at_maxbrake/min_acc; // does not include jerk time
                float stopping_distance = -0.5 * v_at_maxbrake*v_at_maxbrake / min_acc;
                if (dis <= dis_to_maxbrake) {
                    // Case 2.2.1. Arrive the path point before reaching the max deceleration
                    // Solve d(t) = v0*t + a0*t^2/2 -J*t^3/6 = dis to derive the time gap (travel time)
                    auto ts2 = solveRootPolynomial3(-dis, v0, a0/2, -J/6);
                    if (ts2.empty()) {
                        RCLCPP_ERROR(rclcpp::get_logger("TrajectoryManipulation"), "[ERROR] Cannot determine the time gap");
                        break;
                    }
                    time_gap = *std::min_element(ts2.begin(), ts2.end());
                    a1 = a(time_gap, J, a0);
                    v1 = v(time_gap, J, v0, a0);
                } else if (dis <= dis_to_maxbrake + stopping_distance) {
                    // Case 2.2.2.
                    float dis_const_v = dis - dis_to_maxbrake;
                    a1 = min_acc;
                    v1 = std::sqrt(v_at_maxbrake*v_at_maxbrake + 2*min_acc*dis_const_v);
                    time_gap = (v1 - v_at_maxbrake)/min_acc;
                } else { // dis > dis_to_maxbrake + stopping_distance:
                    // Case 2.2.3. Fully stop before reaching the path point
                    a1 = 0.0;
                    v1 = 0.0;
                    time_gap = stopping_time;
                    // In this case the actual travel distance is dis_to_maxbrake + stopping_distance
                }
            }
        }
        new_point.longitudinal_velocity_mps = v1;
        new_point.acceleration_mps2 = a1;
        // new_point.time_from_start.sec = 0;
        // new_point.time_from_start.nanosec = 0;
        new_point.time_from_start.sec = int(time_gap);
        new_point.time_from_start.nanosec = int((time_gap - new_point.time_from_start.sec)*1000000000);

        resampled_points.emplace_back(new_point);
        if (v1 <= 0)
            resampled_points.back().acceleration_mps2 = 0.0;
        i += 1;
    }
    // Implement the resampling logic here
    return resampled_points;
}

/**
find the trajectory point segment on which the $point is located
:return: index i, where trajectory.points[i] is the start of the segment
*/
size_t findSegment(const autoware_planning_msgs::msg::Trajectory& trajectory, 
    const glm::vec2& point) {
    const auto& points = trajectory.points;
    size_t i = 0;
    while (i < points.size() - 1) {
        const auto& start = extractObjPosition(points[i]);
        const auto& end = extractObjPosition(points[i + 1]);
        const auto& segment = start - end;
        const auto& segment2 = point - end;
        if (glm::dot(segment, segment2) >= 0) {
            break;
        }
        i += 1;
    }
    return i;
}

/** 
 * modify the given trajectory to make velocity after stop point 0
 * :param trajectory: original trajectory
 * :param stop_point: desired stop point, np format
 */
autoware_planning_msgs::msg::Trajectory injectMotionVelocityTrajectory(const autoware_planning_msgs::msg::Trajectory& trajectory, const glm::vec2& stop_point) {
    size_t i = findSegment(trajectory, stop_point);
    if (i >= trajectory.points.size() - 1) {
        throw std::runtime_error("Stop point exceeds trajectory length");
    }

    autoware_planning_msgs::msg::Trajectory modified_trajectory = trajectory;
    for (size_t j = i + 1; j < modified_trajectory.points.size(); ++j) {
        modified_trajectory.points[j].longitudinal_velocity_mps = 0.0;
        modified_trajectory.points[j].acceleration_mps2 = 0.0;
    }
    return modified_trajectory;
}