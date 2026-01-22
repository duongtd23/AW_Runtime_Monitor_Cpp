#include "aw_runtime_monitor/planning/trajectory_revision.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

// ============================================================================
// TrajectoryReviser Implementation
// ============================================================================

TrajectoryReviser::TrajectoryReviser() 
    : config_(), logger_(rclcpp::get_logger("trajectory_reviser")) {}

TrajectoryReviser::TrajectoryReviser(const rclcpp::Logger& logger) 
    : config_(), logger_(logger) {}

TrajectoryReviser::TrajectoryReviser(const RevisionConfig& config) 
    : config_(config), logger_(rclcpp::get_logger("trajectory_reviser")) {}

TrajectoryReviser::TrajectoryReviser(const RevisionConfig& config, const rclcpp::Logger& logger) 
    : config_(config), logger_(logger) {}

RevisionResult TrajectoryReviser::revise(
        const autoware_planning_msgs::msg::Trajectory& original_trajectory,
        const std::vector<size_t>& collision_indices,
        const ExternalDrivableAreaChecker& drivable_area_checker,
        const VerifyFunction& verify_func,
        size_t starting_point_id,
        const glm::vec2& ego_size,
        const glm::vec2& ego_center_offset) {
    
    RevisionResult result;
    result.trajectory = original_trajectory;
    
    if (collision_indices.empty()) {
        // No collision points, trajectory should be safe
        result.success = true;
        result.correction_type = CorrectionType::NONE;
        return result;
    }
    
    // Generate candidates sorted by weight
    std::vector<CorrectionCandidate> candidates = generateCandidates(collision_indices);
    
    // Limit number of candidates to try
    size_t max_to_try = std::min(candidates.size(), static_cast<size_t>(config_.max_candidates));

    // cache the revised candidate, which is made with maximum deceleration profile
    autoware_planning_msgs::msg::Trajectory max_decel_candidate_trajectory;
    
    for (size_t i = 0; i < max_to_try; ++i) {
        const CorrectionCandidate& candidate = candidates[i];
        result.candidates_tried++;
        
        std::optional<autoware_planning_msgs::msg::Trajectory> corrected_trajectory = std::nullopt;
        
        switch (candidate.type) {
            case CorrectionType::DECELERATION: {
                corrected_trajectory = applyDecelerationProfile(
                    original_trajectory, candidate.decel_profile, 
                    ego_min_accel_, ego_min_jerk_, starting_point_id);
                if (corrected_trajectory.has_value())
                    max_decel_candidate_trajectory = corrected_trajectory.value();
                break;
            }
            case CorrectionType::LATERAL_SHIFT: {
                corrected_trajectory = applyLateralShift(
                    original_trajectory, collision_indices, candidate.lateral_offset,
                    drivable_area_checker, starting_point_id, ego_size, ego_center_offset);
                break;
            }
            case CorrectionType::COMBINED: {
                // Combined correction: apply lateral shift first, then deceleration
                auto shifted = applyLateralShift(
                    original_trajectory, collision_indices, candidate.lateral_offset,
                    drivable_area_checker, starting_point_id, ego_size, ego_center_offset);
                if (shifted.has_value()) {
                    corrected_trajectory = applyDecelerationProfile(
                        shifted.value(), candidate.decel_profile,
                        ego_min_accel_, ego_min_jerk_, starting_point_id);
                }
                break;
            }
            default:
                continue;
        }
        
        // Skip if correction was not feasible (e.g., outside drivable area)
        if (!corrected_trajectory.has_value()) {
            continue;
        }

        // std::cout << "[DEBUG] Points (x,y, timestamp) of Trajectory candidate #" << (i + 1) << std::endl;
        // for (size_t i = 0; i < std::min(120UL, corrected_trajectory.value().points.size()); ++i) {
        //     const auto& pt = corrected_trajectory.value().points[i];
        //     double ts = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
        //     std::cout << "  (" << pt.pose.position.x << ", " << pt.pose.position.y << ", " << ts << ")," << std::endl;
        // }

        // Verify the corrected trajectory
        if (verify_func(corrected_trajectory.value())) {
            result.success = true;
            result.trajectory = corrected_trajectory.value();
            result.correction_type = candidate.type;
            result.applied_candidate = candidate;

            RCLCPP_INFO(logger_, "Found safe trajectory with candidate #%zu: %s",
                      i + 1, candidate.toString().c_str());

            return result;
        }
    }
    
    // No valid correction found
    result.success = false;
    result.correction_type = CorrectionType::FAILED;
    result.trajectory = max_decel_candidate_trajectory;
    RCLCPP_ERROR(logger_, "Failed to find safe trajectory after trying %u candidates", result.candidates_tried);

    return result;
}

std::vector<CorrectionCandidate> TrajectoryReviser::generateCandidates(
        const std::vector<size_t>& /*collision_indices*/) const {
    
    std::vector<CorrectionCandidate> candidates;
    
    // 1. Deceleration-only candidates (replaces velocity scaling)
    for (const auto& profile : config_.deceleration_profiles) {
        candidates.emplace_back(profile, 0.0f, profile.weight, CorrectionType::DECELERATION);
    }
    
    // 2. Lateral-only candidates (both directions)
    for (size_t i = 0; i < config_.lateral_offsets.size(); ++i) {
        float offset = config_.lateral_offsets[i];
        float weight = config_.lateral_weight_base + i * config_.lateral_weight_factor;
        
        // Positive offset (left)
        candidates.emplace_back(offset, weight, CorrectionType::LATERAL_SHIFT);
        // Negative offset (right)
        candidates.emplace_back(-offset, weight, CorrectionType::LATERAL_SHIFT);
    }
    
    // 3. Combined candidates (deceleration + lateral shift)
    // Combine selected deceleration profiles with selected lateral offsets
    for (int profile_idx : config_.combined_decel_profile_indices) {
        // Check if profile index is valid
        if (profile_idx < 0 || profile_idx >= static_cast<int>(config_.deceleration_profiles.size())) {
            continue;
        }
        
        const auto& profile = config_.deceleration_profiles[profile_idx];
        
        for (size_t i = 0; i < config_.combined_lateral_offsets.size(); ++i) {
            float lateral_offset = config_.combined_lateral_offsets[i];
            
            // Calculate weight as sum of individual weights plus combination penalty
            float decel_weight = profile.weight;
            float lateral_weight = config_.lateral_weight_base + i * config_.lateral_weight_factor;
            float combined_weight = config_.combined_weight_base + 
                                   decel_weight + lateral_weight + 
                                   i * config_.combined_weight_factor;
            
            // Both directions (left and right)
            candidates.emplace_back(profile, lateral_offset, combined_weight, CorrectionType::COMBINED);
            candidates.emplace_back(profile, -lateral_offset, combined_weight + 0.1f, CorrectionType::COMBINED);
        }
    }
    
    // Sort by weight (ascending)
    std::sort(candidates.begin(), candidates.end());
    
    return candidates;
}

autoware_planning_msgs::msg::Trajectory TrajectoryReviser::applyDecelerationProfile(
        const autoware_planning_msgs::msg::Trajectory& trajectory,
        const DecelerationProfile& profile,
        double ego_min_accel,
        double ego_min_jerk,
        size_t starting_point_id) const {
    
    autoware_planning_msgs::msg::Trajectory decelerated = trajectory;
    
    if (starting_point_id >= decelerated.points.size()) {
        return decelerated;
    }
    
    // Calculate actual deceleration and jerk limits based on profile ratios
    double target_decel = ego_min_accel * profile.max_decel_ratio;  // e.g., -7.59 * 0.6 = -4.554 m/s²
    double max_jerk = ego_min_jerk * profile.max_jerk_ratio;        // e.g., -12.65 * 0.6 = -7.59 m/s³
    
    // Get initial velocity at starting point
    double v0 = decelerated.points[starting_point_id].longitudinal_velocity_mps;
    double a0 = decelerated.points[starting_point_id].acceleration_mps2;
    
    // Calculate target velocity
    double v_target = v0 * profile.target_speed_ratio;
    
    // Minimum speed threshold to consider as stopped
    const double min_speed_threshold = 0.01;  // m/s
    if (v_target < min_speed_threshold) {
        v_target = 0.0;
    }
    
    // If already at or below target speed, no deceleration needed
    if (v0 <= v_target) {
        return decelerated;
    }
    
    RCLCPP_INFO(logger_, "Applying profile '%s'", profile.name.c_str());
    
    // Apply jerk-limited deceleration using S-curve profile
    // Phase 1: Jerk-in (increase deceleration magnitude from a0 to target_decel)
    // Phase 2: Constant deceleration (maintain target_decel)
    // Phase 3: Jerk-out (decrease deceleration to 0 when approaching target velocity)
    
    double current_velocity = v0;
    double current_accel = a0;
    
    for (size_t i = starting_point_id; i < decelerated.points.size(); ++i) {
        auto& point = decelerated.points[i];
        
        // If we've reached target velocity, set remaining points to target velocity
        if (current_velocity <= v_target) {
            point.longitudinal_velocity_mps = v_target;
            point.acceleration_mps2 = 0.0;
            continue;
        }
        
        // Calculate time step to next point
        double dt = 0.1;  // Default time step
        if (i < decelerated.points.size() - 1) {
            glm::vec2 pos_curr(point.pose.position.x, point.pose.position.y);
            glm::vec2 pos_next(decelerated.points[i + 1].pose.position.x, 
                              decelerated.points[i + 1].pose.position.y);
            double distance = glm::length(pos_next - pos_curr);
            if (current_velocity > 0.01) {
                dt = distance / current_velocity;
            }
        }
        
        // Determine next acceleration based on current phase
        double next_accel = current_accel;
        
        // Calculate velocity needed to stop with jerk-out from current acceleration
        // During jerk-out: a(t) = a_current - jerk * t, until a = 0
        // Time for jerk-out: t_jerk = |a_current / jerk|
        // Velocity change during jerk-out: Δv = a_current * t_jerk - 0.5 * jerk * t_jerk^2
        //                                     = a_current^2 / (2 * |jerk|)
        double jerk_out_time = std::abs(current_accel / max_jerk);
        double velocity_change_during_jerkout = 0.5 * current_accel * jerk_out_time;
        double velocity_after_jerkout = current_velocity + velocity_change_during_jerkout;
        
        // Decide which phase to apply
        if (velocity_after_jerkout <= v_target) {
            // Phase 3: Jerk-out - we're close enough to target, start reducing deceleration
            next_accel = current_accel - max_jerk * dt;
            
            // Don't let acceleration become positive (no acceleration, only deceleration or zero)
            if (next_accel > 0.0) {
                next_accel = 0.0;
            }
        }
        else if (current_accel > target_decel) {
            // Phase 1: Jerk-in - increase deceleration magnitude
            next_accel = current_accel + max_jerk * dt;
            
            // Clamp to target deceleration
            if (next_accel < target_decel) {
                next_accel = target_decel;
            }
        }
        else {
            // Phase 2: Constant deceleration - maintain target deceleration
            next_accel = target_decel;
        }
        
        // Update velocity using average acceleration over the time step (trapezoidal integration)
        double avg_accel = 0.5 * (current_accel + next_accel);
        double next_velocity = current_velocity + avg_accel * dt;
        
        // Ensure we don't go below target velocity
        if (next_velocity < v_target) {
            next_velocity = v_target;
            next_accel = 0.0;
        }
        
        // Ensure non-negative velocity
        if (next_velocity < 0.0) {
            next_velocity = 0.0;
            next_accel = 0.0;
        }
        
        // Update point
        point.longitudinal_velocity_mps = next_velocity;
        point.acceleration_mps2 = next_accel;
        
        // Update for next iteration
        current_velocity = next_velocity;
        current_accel = next_accel;
    }
    
    // Update time_from_start based on new velocities
    updateTimeFromStart(decelerated, starting_point_id);
    
    return decelerated;
}

std::optional<autoware_planning_msgs::msg::Trajectory> TrajectoryReviser::applyLateralShift(
        const autoware_planning_msgs::msg::Trajectory& trajectory,
        const std::vector<size_t>& collision_indices,
        float lateral_offset,
        const ExternalDrivableAreaChecker& drivable_area_checker,
        size_t starting_point_id,
        const glm::vec2& ego_size,
        const glm::vec2& ego_center_offset) const {

    if (collision_indices.empty()) {
        return trajectory;
    }
    
    autoware_planning_msgs::msg::Trajectory shifted = trajectory;
    
    // Determine the range of points to shift (collision points + neighbors for smoothing)
    size_t min_collision_idx = *std::min_element(collision_indices.begin(), collision_indices.end());
    size_t max_collision_idx = *std::max_element(collision_indices.begin(), collision_indices.end());
    
    // Extend range for smooth transition (but not before starting_point_id)
    int points_before_collision = min_collision_idx - starting_point_id;
    int desired_prior_points = (config_.min_points - (max_collision_idx - min_collision_idx))/2;
    if (desired_prior_points > points_before_collision) {
        desired_prior_points = points_before_collision;
    }

    size_t shift_start = min_collision_idx - desired_prior_points;
    size_t shift_end = std::max(max_collision_idx, shift_start + config_.min_points);

    // RCLCPP_INFO(logger_, "Shift range: [%zu, %zu] (min_collision_idx: %zu, max_collision_idx: %zu)",
                // shift_start, shift_end, min_collision_idx, max_collision_idx);
    
    // Create a set of collision indices for quick lookup
    std::set<size_t> collision_set(collision_indices.begin(), collision_indices.end());
    
    // Apply lateral shift with tapering
    for (size_t i = shift_start; i < shift_end; ++i) {
        auto& point = shifted.points[i];
        
        // Compute taper factor (1.0 at collision points, decreasing towards edges)
        float taper = 1.0f;
        if (i < min_collision_idx) {
            // Before collision zone - taper in
            taper = 1.0f - static_cast<float>(min_collision_idx - i) / 
                    static_cast<float>(min_collision_idx - shift_start + 1);
        } else if (i > max_collision_idx) {
            // After collision zone - taper out
            taper = 1.0f - static_cast<float>(i - max_collision_idx) / 
                    static_cast<float>(shift_end - max_collision_idx);
        }
        
        // Get heading at this point
        auto euler = quaternionToEulerAngles(point.pose.orientation);
        float heading = static_cast<float>(euler[2]);
        
        // Compute lateral direction
        glm::vec2 lateral_dir = getLateralDirection(heading);
        
        // Apply tapered offset
        float actual_offset = lateral_offset * taper;
        point.pose.position.x += actual_offset * lateral_dir.x;
        point.pose.position.y += actual_offset * lateral_dir.y;
        
        // Check if point is still within drivable area
        if (!drivable_area_checker.isPointInDrivableArea(
                point.pose.position.x, point.pose.position.y)) {
            // Point outside drivable area - this shift is not feasible
            RCLCPP_WARN(logger_, "Lateral shift resulted in point outside drivable area at index %zu", i);
            return std::nullopt;
        }
    }
    
    // Apply Elastic Band smoothing
    applyElasticBandSmoothing(shifted.points, shift_start, shift_end);

    // std::cout << "[DEBUG] Laterally shifted points are (index, x,y):" << std::endl;
    // for (size_t i = shift_start; i < shift_end; ++i) {
    //     const auto& point = shifted.points[i];
    //     std::cout << "  (" << i << ", " << point.pose.position.x << ", " << point.pose.position.y << ")," << std::endl;
    // }
    
    // Check curvature constraints
    if (!checkCurvatureConstraints(shifted.points, shift_start)) {
        RCLCPP_WARN(logger_, "Lateral shift resulted in curvature constraint violation.");
        return std::nullopt;
    }
    
    // Update headings based on new path
    updateHeadings(shifted.points, shift_start);

    // Verify all shifted points are still in drivable area after smoothing
    // Need to check the vehicle vertices, not the point itself
    for (size_t i = shift_start; i < shift_end; ++i) {
        const auto& point = shifted.points[i];
        // Get heading at this point
        auto euler = quaternionToEulerAngles(point.pose.orientation);
        std::vector<glm::vec2> ego_vertices;
        if (lateral_offset > 0) {
            // Left shift
            ego_vertices = getEgoRightVertices(
                        glm::vec2(point.pose.position.x, point.pose.position.y), 
                        euler[2], ego_size, ego_center_offset);
        } else {
            // Right shift
            ego_vertices = getEgoLeftVertices(
                        glm::vec2(point.pose.position.x, point.pose.position.y), 
                        euler[2], ego_size, ego_center_offset);
        }
        if (!drivable_area_checker.isPointInDrivableArea(
                ego_vertices[0].x, ego_vertices[0].y) ||
            !drivable_area_checker.isPointInDrivableArea(
                ego_vertices[1].x, ego_vertices[1].y)) {
            RCLCPP_WARN(logger_, "A point outside drivable area at index %zu", i);
            return std::nullopt;
        }
    }

    // Update time_from_start
    updateTimeFromStart(shifted, starting_point_id);
    
    return shifted;
}

void TrajectoryReviser::applyElasticBandSmoothing(
        std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points,
        size_t fixed_start_idx,
        size_t fixed_end_idx) const {
    
    if (fixed_end_idx <= fixed_start_idx + 2) {
        return; // Not enough points to smooth
    }
    
    // Elastic Band smoothing iterations
    // Each point is pulled towards the midpoint of its neighbors
    for (int iter = 0; iter < config_.smoothing_iterations; ++iter) {
        std::vector<glm::vec2> new_positions(points.size());
        
        // Copy current positions
        for (size_t i = 0; i < points.size(); ++i) {
            new_positions[i] = glm::vec2(points[i].pose.position.x, points[i].pose.position.y);
        }
        
        // Apply smoothing to interior points only
        for (size_t i = fixed_start_idx + 1; i < fixed_end_idx - 1; ++i) {
            glm::vec2 prev(points[i - 1].pose.position.x, points[i - 1].pose.position.y);
            glm::vec2 curr(points[i].pose.position.x, points[i].pose.position.y);
            glm::vec2 next(points[i + 1].pose.position.x, points[i + 1].pose.position.y);
            
            // Midpoint of neighbors
            glm::vec2 midpoint = 0.5f * (prev + next);
            
            // Move current point towards midpoint (internal force)
            glm::vec2 smoothed = curr + (float)config_.smoothing_weight * (midpoint - curr);
            
            new_positions[i] = smoothed;
        }
        
        // Update positions
        for (size_t i = fixed_start_idx + 1; i < fixed_end_idx - 1; ++i) {
            points[i].pose.position.x = new_positions[i].x;
            points[i].pose.position.y = new_positions[i].y;
        }
    }
}

void TrajectoryReviser::updateTimeFromStart(
        autoware_planning_msgs::msg::Trajectory& trajectory,
        size_t starting_point_id) const {
    
    if (trajectory.points.empty()) {
        return;
    }
    
    // Set initial time at starting_point_id to 0
    if (starting_point_id < trajectory.points.size()) {
        trajectory.points[starting_point_id].time_from_start.sec = 0;
        trajectory.points[starting_point_id].time_from_start.nanosec = 0;
    }
    
    float cumulative_time = 0.0f;
    
    for (size_t i = starting_point_id + 1; i < trajectory.points.size(); ++i) {
        const auto& prev_point = trajectory.points[i - 1];
        const auto& curr_point = trajectory.points[i];
        
        glm::vec2 pos_prev(prev_point.pose.position.x, prev_point.pose.position.y);
        glm::vec2 pos_curr(curr_point.pose.position.x, curr_point.pose.position.y);
        
        float distance = glm::length(pos_curr - pos_prev);
        float avg_velocity = 0.5f * (prev_point.longitudinal_velocity_mps + 
                                      curr_point.longitudinal_velocity_mps);
        
        float dt = 0.0f;
        if (avg_velocity > 0.01f) {
            dt = distance / avg_velocity;
        } else {
            // If velocity is near zero, use a small default time step
            dt = 0.1f;
        }
        
        cumulative_time += dt;
        
        // Convert to ROS duration
        int32_t sec = static_cast<int32_t>(cumulative_time);
        uint32_t nanosec = static_cast<uint32_t>((cumulative_time - sec) * 1e9);
        
        trajectory.points[i].time_from_start.sec = sec;
        trajectory.points[i].time_from_start.nanosec = nanosec;
    }
}

void TrajectoryReviser::updateHeadings(
        std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points,
        size_t start_idx) const {
    
    for (size_t i = start_idx; i < points.size(); ++i) {
        glm::vec2 curr(points[i].pose.position.x, points[i].pose.position.y);
        glm::vec2 next;
        
        if (i < points.size() - 1) {
            next = glm::vec2(points[i + 1].pose.position.x, points[i + 1].pose.position.y);
        } else if (i > 0) {
            // For last point, use direction from previous point
            glm::vec2 prev(points[i - 1].pose.position.x, points[i - 1].pose.position.y);
            next = curr + (curr - prev);  // Extrapolate
        } else {
            continue; // Single point, keep original heading
        }
        
        float heading = computeHeading(curr, next);
        points[i].pose.orientation = yawToQuaternion(heading);
    }
}

bool TrajectoryReviser::checkCurvatureConstraints(
        const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points,
        size_t start_idx) const {
    
    for (size_t i = start_idx + 1; i < points.size() - 1; ++i) {
        glm::vec2 p1(points[i - 1].pose.position.x, points[i - 1].pose.position.y);
        glm::vec2 p2(points[i].pose.position.x, points[i].pose.position.y);
        glm::vec2 p3(points[i + 1].pose.position.x, points[i + 1].pose.position.y);
        
        float curvature = computeCurvature(p1, p2, p3);
        
        if (std::abs(curvature) > config_.max_curvature) {
            return false;
        }
    }
    
    return true;
}

float TrajectoryReviser::computeCurvature(
        const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3) const {
    
    // Menger curvature: k = 4 * Area / (|p1-p2| * |p2-p3| * |p1-p3|)
    float a = glm::length(p2 - p1);
    float b = glm::length(p3 - p2);
    float c = glm::length(p3 - p1);
    
    // Area of triangle using cross product
    float cross = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
    float area = 0.5f * std::abs(cross);
    
    float denom = a * b * c;
    if (denom < 1e-9f) {
        return 0.0f; // Degenerate case
    }
    
    return 4.0f * area / denom;
}