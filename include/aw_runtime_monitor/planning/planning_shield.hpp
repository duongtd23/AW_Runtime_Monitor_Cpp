#ifndef PLANNING_SHIELD_HPP
#define PLANNING_SHIELD_HPP

#include "aw_runtime_monitor/planning/trajectory_manipulation.hpp"
#include "aw_runtime_monitor/planning/planning_trajectory.hpp"
#include "aw_runtime_monitor/planning/drivable_area_checker.hpp"
#include "aw_runtime_monitor/planning/trajectory_revision.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "aw_runtime_monitor/groundtruth/groundtruth_size.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter_client.hpp>
#include <spot/tl/parse.hh>
#include <spot/kripke/kripkegraph.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/emptiness.hh>
#include <functional>  // for std::function
#include <map>
#include <tuple>

// for CLI interaction with Maude
#include <unistd.h>
#include <pty.h>
#include <sys/wait.h>

using ComparisonFunction = std::function<bool(float)>;

class PlanningShield 
{
public:
    PlanningShield() : logger_(rclcpp::get_logger("planning_shield")) {}
    PlanningShield(const std::string& safety_formula_str, 
                   const std::string& spec_syntax_file_path,
                   float speed_threshold_activation,
                   float time_bound,
                   float distance_bound,
                   const RevisionConfig& revision_config);

    struct VerificationResult {
        bool is_safe; // if the original trajectory is safe
        bool was_revised; // if the trajectory was revised
        autoware_planning_msgs::msg::Trajectory revised_msg; // or original msg if safe
    };
    struct InternalVerificationResult {
        bool is_safe;
        std::vector<size_t> collision_indices; // indices of trajectory points that have collision
    };

    /**
     * @brief Verify the planned trajectory against the safety specification, using the detected objects from recorded data
     * @param trajectory_msg The original planned trajectory message
     * @param recorded_data The recorded data containing estimated kinematic and perception objects
     * @param enable_revision Whether to enable trajectory revision
     * @return VerificationResult containing the safety status and revised trajectory if applicable
     */
    VerificationResult verify(const autoware_planning_msgs::msg::Trajectory& trajectory_msg, 
                const nlohmann::json& recorded_data,
                bool enable_revision = true);

    InternalVerificationResult doVerify(
        const autoware_planning_msgs::msg::Trajectory& trajectory_msg,
        const nlohmann::json& recorded_data,
        size_t starting_point_id,
        // const glm::vec3& current_position, // current ego position
        const glm::vec2& ego_size,
        const glm::vec2& ego_center_offset);

    void setMapForDrivableAreaChecker(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg) {
        drivable_area_checker_.setMap(map_msg);
    }
    void setRouteForDrivableAreaChecker(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_msg) {
        drivable_area_checker_.setRoute(route_msg);
    }

    void clearRouteForDrivableAreaChecker() {
        drivable_area_checker_.clearRoute();
    }

    ExternalDrivableAreaChecker::DrivableAreaBounds getExpandedDrivableArea(
        geometry_msgs::msg::Pose current_pose,
        const double backward_distance = 5.0,
        const double forward_distance = 80.0) {
        return drivable_area_checker_.getExpandedDrivableArea(current_pose, backward_distance, forward_distance);
    }

    /**
     * @brief Handle scenario planning trajectory.
     * If the latest trajectory was revised, update the speed of the scenario planning trajectory points.
     */
    std::optional<autoware_planning_msgs::msg::Trajectory> handleScenarioPlanningTrajectory(
        const autoware_planning_msgs::msg::Trajectory& scenario_planning_trajectory,
        const nlohmann::json& recorded_data);

    void setMinAccJerk(double min_acc, double min_jerk) {
        ego_min_accel_ = min_acc;
        ego_min_jerk_ = min_jerk;
        trajectory_reviser_.setVehicleDynamics(min_acc, min_jerk);
    }
private:
    // specification of the desired safety requirements
    spot::parsed_formula spec_formula_;
    std::string spec_syntax_file_path_;
    // list of atomic propositions in the specification
    std::vector<std::string> propositions_;
    // mapping from proposition string to evaluation function
    std::map<std::string, ComparisonFunction> proposition_map_;

    // Each point in a scenario planning trajectory has only two possible speeds:
    // either zero (stop) or this speed value (normally the max speed limit of the road)
    // this value will be updated in runtime based on the scenario planning trajectory received
    double scenario_planning_trajectory_speed;

    // Latest verification result of the planning trajectory
    VerificationResult latest_verification_result_;

    // Minimum acceleration and jerk for the ego vehicle (always negative values)
    // These values are obtained from Autoware (shield does not decide their values)
    // Standard values are: min_acc = -7.59 m/s^2, min_jerk = -12.65 m/s^3
    double ego_min_accel_, ego_min_jerk_;

    // don't activate the shield when ego speed is below this threshold
    float speed_threshold_activation_;
    // ignore trajectory points beyond this time bound (to boost the performance)
    float time_bound_;
    // ignore objects farther than this distance bound (to boost the performance)
    float distance_bound_;

    // Drivable area checker
    ExternalDrivableAreaChecker drivable_area_checker_;
    
    // Trajectory reviser for correcting unsafe trajectories
    TrajectoryReviser trajectory_reviser_;

    spot::bdd_dict_ptr cached_bdd_dict_;

    rclcpp::Logger logger_;

    bool evaluateSpec(const std::vector<float>& timestamp_series,
        const std::vector<bool>& collision_series,
        const std::vector<float>& distance_series,
        float existence_prob,
        float path_confidence);
};

// ============================================================================
// Helper functions
// ============================================================================

/**
 * @brief Extract the closest trajectory point to the current ego position.
 * This is because a trajectory also includes some points behind the ego vehicle.
 * @param raw_points The raw trajectory points.
 * @param current_ego_position The current ego position.
 * @return A tuple of the index of the closest point and the distance to it.
 */
inline std::tuple<size_t, float> extractClosestPoint(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint>& raw_points, const glm::vec2& current_ego_position) {
    float min_dis = 1e9;
    size_t _id = -1;
    for (size_t i = 0; i < raw_points.size(); ++i) {
        const auto& point = raw_points[i];
        glm::vec2 nppoint = extractObjPosition(point);
        float dis = glm::length(current_ego_position - nppoint);
        if (dis < min_dis) {
            _id = i;
            min_dis = dis;
        }
    }
    return std::make_tuple(_id, min_dis);
}

/**
 * Helper functions for PlanningShield class
 */
// Send a Maude command
inline void sendCommand(int fd, const std::string& cmd) {
    std::string full_cmd = cmd + "\n";
    write(fd, full_cmd.c_str(), full_cmd.size());
}
// Wait until a specific prompt string appears in the Maude output
inline bool waitForPrompt(int fd, const std::string& prompt, std::string* captured_output = nullptr) {
    std::string buffer;
    char ch;
    while (read(fd, &ch, 1) > 0) {
        buffer += ch;
        if (buffer.size() >= prompt.size() &&
            buffer.compare(buffer.size() - prompt.size(), prompt.size(), prompt) == 0) {
            if (captured_output)
                *captured_output = buffer;
            return true;
        }
    }
    return false;
}

/**
 * Rename formula via Maude CLI
 * E.g.,
 * Return a vector of string, the first one is the renamed formula, and
 * the rest are the renamed propositions.
 */
inline std::vector<std::string> renameFormula(const std::string& formula_str, 
        const std::string& spec_syntax_file_path) {
    int master_fd;
    pid_t pid = forkpty(&master_fd, nullptr, nullptr, nullptr);

    if (pid == 0) {
        // Child process: execute Maude
        execlp("maude", "maude", (char*)nullptr);
        perror("execlp failed");
        return {};
    }

    // Parent process
    if (!waitForPrompt(master_fd, "Maude>")) {
        std::cerr << "Failed to start Maude." << std::endl;
        return {};
    }
    sendCommand(master_fd, "load " + spec_syntax_file_path);

    if (!waitForPrompt(master_fd, "Maude>")) {
        std::cerr << "Failed to load model." << std::endl;
        return {};
    }

    // std::cout << "\n[Model loaded successfully]\n";
    sendCommand(master_fd, "red rename(" + formula_str + ") .");

    std::string reduction_output;
    std::vector<std::string> result = {};
    if (waitForPrompt(master_fd, "Maude>", &reduction_output)) {
        // std::cout << "\n=== Reduction Output ===\n";
        size_t pos = reduction_output.find("Maude>");
        if (pos != std::string::npos)
            reduction_output.erase(pos);
        // std::cout << reduction_output << std::endl;
        std::string result_str = extractResultFromOutput(reduction_output);
        if (result_str == "") {
            std::cerr << "A syntax error with formula: " << formula_str << std::endl;
            std::cerr << "Reduction output from Maude:\n" << reduction_output << std::endl;
        }
        else {
            result = toVectorString(result_str);
        }
        // std::cout << "\n[Extracted Result]:" << result << std::endl;
    } else {
        std::cerr << "No reduction result captured." << std::endl;
    }

    // Quit Maude
    sendCommand(master_fd, "quit");
    waitpid(pid, nullptr, 0);
    return result;
}

inline glm::vec2 correctVelocity(const nlohmann::json& perception_msgs,
        const glm::vec2& current_pos, float current_time,
        const std::string& object_id, size_t bounded_entry_id,
        size_t lookback_steps=2) {
    glm::vec2 prev_position;
    float time = 0.0;
    size_t steps = 0;
    for (size_t i = bounded_entry_id - 1; i < perception_msgs.size(); --i) {
        const auto& perp_entry = perception_msgs[i];
        for (const auto& obj : perp_entry["objects"]) {
            if (obj["id"] == object_id) {
                prev_position = jsonPointToVector2(obj["pose"]["position"]);
                time = perp_entry["timestamp"];
                steps++;
            }
        }
        if (steps >= lookback_steps) break;
    }

    float dt = current_time - time;
    if (steps < lookback_steps || dt >= 0.6f) {
        return glm::vec2{0.0, 0.0};
    }
    return (current_pos - prev_position) / dt;
}

inline float deriveTimeStep(const glm::vec2& position0, float speed0, float accel0, 
                    const glm::vec2& position1, float speed1, float accel1){
    if (accel0 + accel1 == 0.0) {
        if (speed0 + speed1 == 0.0) {
            return 0.0;
        }
        return 2 * float(glm::length(position1 - position0)) / (speed0 + speed1);
    }
    float temp = 2 * (speed1 - speed0) / (accel0 + accel1);
    if (temp < 0) {
        return 2 * float(glm::length(position1 - position0)) / (speed0 + speed1);
    }
    return temp;
}

#endif