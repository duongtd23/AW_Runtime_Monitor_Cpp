#ifndef TRAJECTORY_REVISION_HPP
#define TRAJECTORY_REVISION_HPP

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "aw_runtime_monitor/utils.hpp"
#include "aw_runtime_monitor/planning/drivable_area_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <vector>
#include <functional>
#include <optional>

/**
 * @brief Structure representing a deceleration profile for trajectory revision
 */
struct DecelerationProfile {
    std::string name;              // Profile name (e.g., "gentle_stop")
    double target_speed_ratio;     // Target speed ratio (0.0 = full stop, 0.3 = 30% speed)
    double max_decel_ratio;        // Ratio of ego_min_accel to use (0.5 = 50%, 1.0 = 100%)
    double max_jerk_ratio;         // Ratio of ego_min_jerk to use (0.5 = 50%, 1.0 = 100%)
    double weight;                 // Weight for candidate ordering (lower = preferred)
    
    DecelerationProfile() 
        : name(""), target_speed_ratio(0.0), max_decel_ratio(1.0), 
          max_jerk_ratio(1.0), weight(1.0) {}
    
    DecelerationProfile(const std::string& n, double tsr, double mdr, double mjr, double w)
        : name(n), target_speed_ratio(tsr), max_decel_ratio(mdr), 
          max_jerk_ratio(mjr), weight(w) {}
};

/**
 * @brief Enumeration of correction strategy types
 */
enum class CorrectionType {
    NONE,                   // No correction applied
    DECELERATION,           // Jerk-limited deceleration profile applied
    LATERAL_SHIFT,          // Only lateral position shifted (Elastic Band)
    COMBINED,               // Both deceleration and lateral shift
    FAILED                  // No valid correction found
};

/**
 * @brief Structure representing a correction candidate
 */
struct CorrectionCandidate {
    DecelerationProfile decel_profile;  // Deceleration profile (for DECELERATION type)
    float lateral_offset;               // Lateral offset in meters (positive = left, negative = right)
    float weight;                       // Weight/cost of this correction (lower = preferred)
    CorrectionType type;                // Type of correction
    
    // Constructor for deceleration-based correction
    CorrectionCandidate(const DecelerationProfile& profile, float lat_offset, float w, CorrectionType t)
        : decel_profile(profile), lateral_offset(lat_offset), weight(w), type(t) {}
    
    // Constructor for lateral-only correction (backward compatibility)
    CorrectionCandidate(float lat_offset, float w, CorrectionType t)
        : decel_profile(), lateral_offset(lat_offset), weight(w), type(t) {}
    
    // Comparison operator for sorting by weight
    bool operator<(const CorrectionCandidate& other) const {
        return weight < other.weight;
    }

    std::string toString() const {
        switch (type) {
            case CorrectionType::DECELERATION:
                return "Deceleration, profile = " + decel_profile.name;
            case CorrectionType::LATERAL_SHIFT:
                return "Lateral shift, offset = " + std::to_string(lateral_offset);
            case CorrectionType::COMBINED:
                return "Combined, decel_profile = " + decel_profile.name + ", lateral_offset = " + std::to_string(lateral_offset);
            default:
                return "none";
        }
    }
};

/**
 * @brief Result of trajectory revision
 */
struct RevisionResult {
    bool success;                                              // Whether a valid revision was found
    autoware_planning_msgs::msg::Trajectory trajectory;        // The revised trajectory
    CorrectionType correction_type;                            // Type of correction applied
    CorrectionCandidate applied_candidate;                     // The candidate that was applied
    int candidates_tried;                                      // Number of candidates evaluated
    
    RevisionResult() 
        : success(false), correction_type(CorrectionType::FAILED),
          applied_candidate(DecelerationProfile(), 0.0f, 0.0f, CorrectionType::NONE), candidates_tried(0) {}
};

/**
 * @brief Configuration for trajectory revision
 */
struct RevisionConfig {
    // Deceleration profiles (replaces velocity scaling)
    std::vector<DecelerationProfile> deceleration_profiles = {};

    // Lateral shift parameters
    std::vector<double> lateral_offsets = {};
    double lateral_weight_base = 5.0;       // Base weight for lateral shift (higher than deceleration)
    double lateral_weight_factor = 3.0;     // Weight multiplier per offset step

    // Combined correction parameters
    // Indices of deceleration profiles to use in combined corrections
    std::vector<long int> combined_decel_profile_indices = {0, 1, 2};  // e.g., gentle, moderate, aggressive
    // Lateral offsets to use in combined corrections (in meters)
    std::vector<double> combined_lateral_offsets = {0.5, 1.0, 1.5};
    double combined_weight_base = 10.0;     // Base weight for combined (higher than individual)
    double combined_weight_factor = 2.0;    // Weight multiplier per combination

    // Elastic Band smoothing parameters
    int smoothing_iterations = 5;           // Number of smoothing iterations
    double smoothing_weight = 0.3;          // Weight for neighbor influence (0-0.5)

    // Trajectory constraints
    double max_curvature = 1.0;             // Maximum allowed curvature (1/m)

    // Search limits
    int max_candidates = 50;                // Maximum number of candidates to try

    // minimum number of points to be revised (in order to make smooth trajectory)
    int min_points = 30;
};

/**
 * @brief Class for revising unsafe trajectories using weighted candidate enumeration
 * 
 * Implements two main correction strategies:
 * 1. Velocity scaling: Reduce velocity and acceleration to slow down
 * 2. Lateral shift (Elastic Band): Shift trajectory points laterally to avoid collision
 * 
 * Candidates are enumerated by weight (lower = preferred) until a safe trajectory is found.
 */
class TrajectoryReviser {
public:
    using VerifyFunction = std::function<bool(const autoware_planning_msgs::msg::Trajectory&)>;
    
    TrajectoryReviser();
    explicit TrajectoryReviser(const rclcpp::Logger& logger);
    explicit TrajectoryReviser(const RevisionConfig& config);
    TrajectoryReviser(const RevisionConfig& config, const rclcpp::Logger& logger);
    
    /**
     * @brief Revise an unsafe trajectory to satisfy safety requirements
     * @param original_trajectory The original unsafe trajectory
     * @param collision_indices Indices of trajectory points that have collision
     * @param drivable_area_checker Checker to verify points are within drivable area
     * @param verify_func Function to verify if a trajectory is safe
     * @param starting_point_id Index of the closest point to ego (points before this are unchanged)
     * @return RevisionResult containing the revised trajectory and metadata
     */
    RevisionResult revise(
        const autoware_planning_msgs::msg::Trajectory& original_trajectory,
        const std::vector<size_t>& collision_indices,
        const ExternalDrivableAreaChecker& drivable_area_checker,
        const VerifyFunction& verify_func,
        size_t starting_point_id,
        const glm::vec2& ego_size,
        const glm::vec2& ego_center_offset);

    /**
     * @brief Generate ordered list of correction candidates by weight
     * @param collision_indices Indices of points with collision (used to determine shift direction)
     * @return Vector of candidates sorted by weight (ascending)
     */
    std::vector<CorrectionCandidate> generateCandidates(
        const std::vector<size_t>& collision_indices) const;
    
    /**
     * @brief Apply jerk-limited deceleration profile to trajectory
     * @param trajectory Original trajectory
     * @param profile Deceleration profile to apply
     * @param ego_min_accel Minimum acceleration of ego vehicle (negative value, e.g., -7.59 m/s²)
     * @param ego_min_jerk Minimum jerk of ego vehicle (negative value, e.g., -12.65 m/s³)
     * @param starting_point_id Index from which to apply deceleration
     * @return Trajectory with deceleration profile applied
     */
    autoware_planning_msgs::msg::Trajectory applyDecelerationProfile(
        const autoware_planning_msgs::msg::Trajectory& trajectory,
        const DecelerationProfile& profile,
        double ego_min_accel,
        double ego_min_jerk,
        size_t starting_point_id) const;
    
    /**
     * @brief Apply lateral shift using Elastic Band algorithm
     * @param trajectory Original trajectory
     * @param collision_indices Indices of points to shift
     * @param lateral_offset Lateral offset in meters
     * @param drivable_area_checker Checker to verify points are within drivable area
     * @param starting_point_id Index from which shifting is allowed
     * @return Shifted trajectory with smoothing applied, or nullopt if infeasible
     */
    std::optional<autoware_planning_msgs::msg::Trajectory> applyLateralShift(
        const autoware_planning_msgs::msg::Trajectory& trajectory,
        const std::vector<size_t>& collision_indices,
        float lateral_offset,
        const ExternalDrivableAreaChecker& drivable_area_checker,
        size_t starting_point_id,
        const glm::vec2& ego_size,
        const glm::vec2& ego_center_offset) const;

    // Configuration access
    const RevisionConfig& getConfig() const { return config_; }
    void setConfig(const RevisionConfig& config) { config_ = config; }
    
    // Vehicle dynamics constraints
    void setVehicleDynamics(double min_accel, double min_jerk) {
        ego_min_accel_ = min_accel;
        ego_min_jerk_ = min_jerk;
    }
    double getMinAccel() const { return ego_min_accel_; }
    double getMinJerk() const { return ego_min_jerk_; }

private:
    RevisionConfig config_;
    rclcpp::Logger logger_;
    
    // Vehicle dynamics constraints
    double ego_min_accel_ = -7.59;   // Minimum acceleration (m/s²) - default value
    double ego_min_jerk_ = -12.65;   // Minimum jerk (m/s³) - default value
    
    /**
     * @brief Apply Elastic Band smoothing to trajectory
     * @param points Trajectory points to smooth
     * @param fixed_start_idx Index of first point that can be modified
     * @param fixed_end_idx Index of last point that can be modified (exclusive)
     */
    void applyElasticBandSmoothing(
        std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points,
        size_t fixed_start_idx,
        size_t fixed_end_idx) const;
    
    /**
     * @brief Update time_from_start for all trajectory points
     * @param trajectory Trajectory to update
     * @param starting_point_id Index from which to start updating
     */
    void updateTimeFromStart(
        autoware_planning_msgs::msg::Trajectory& trajectory,
        size_t starting_point_id) const;
    
    /**
     * @brief Update heading (orientation) based on path direction
     * @param points Trajectory points to update
     * @param start_idx Index from which to update
     */
    void updateHeadings(
        std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points,
        size_t start_idx) const;
    
    /**
     * @brief Check if trajectory satisfies curvature constraints
     * @param points Trajectory points to check
     * @param start_idx Index from which to check
     * @return True if curvature is within bounds
     */
    bool checkCurvatureConstraints(
        const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& points,
        size_t start_idx) const;
    
    /**
     * @brief Compute curvature at a point using three consecutive points
     * @param p1 Previous point
     * @param p2 Current point
     * @param p3 Next point
     * @return Curvature value (1/radius)
     */
    float computeCurvature(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3) const;
    
    /**
     * @brief Determine preferred lateral shift direction based on collision geometry
     * @param trajectory Original trajectory
     * @param collision_indices Indices of collision points
     * @return Positive for left shift preferred, negative for right shift preferred
     */
    // float determineShiftDirection(
    //     const autoware_planning_msgs::msg::Trajectory& trajectory,
    //     const std::vector<size_t>& collision_indices) const;
};

// ============================================================================
// Helper functions
// ============================================================================

/**
 * @brief Convert Euler angle (yaw in degrees) to quaternion
 */
inline geometry_msgs::msg::Quaternion yawToQuaternion(float yaw_degrees) {
    float yaw_rad = glm::radians(yaw_degrees);
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw_rad / 2.0f);
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = std::sin(yaw_rad / 2.0f);
    return q;
}

/**
 * @brief Compute heading angle from two consecutive points
 * @param from Starting point
 * @param to Ending point
 * @return Heading angle in degrees
 */
inline float computeHeading(const glm::vec2& from, const glm::vec2& to) {
    glm::vec2 direction = to - from;
    if (glm::length(direction) < 1e-6f) {
        return 0.0f;
    }
    return glm::degrees(std::atan2(direction.y, direction.x));
}

/**
 * @brief Get lateral direction (perpendicular to heading)
 * @param heading_degrees Heading angle in degrees
 * @return Unit vector perpendicular to heading (pointing left)
 */
inline glm::vec2 getLateralDirection(float heading_degrees) {
    float heading_rad = glm::radians(heading_degrees);
    // Perpendicular to heading: rotate 90 degrees counter-clockwise
    return glm::vec2(-std::sin(heading_rad), std::cos(heading_rad));
}

#endif // TRAJECTORY_REVISION_HPP
