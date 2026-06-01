#ifndef FQLTL_DATA_OBJECT_HPP
#define FQLTL_DATA_OBJECT_HPP

#include <string>
#include <ostream>
#include <unordered_map>
#include <variant>
#include <optional>
#include <vector>
#include <glm/glm.hpp>  // for vector computation
#include <glm/gtx/string_cast.hpp>

/// @brief Data structures for representing the runtime data objects (ego trajectory, perceived objects, predicted paths) used in FQLTL evaluation.

namespace fqltl {

/**
 * @brief Enumeration of object classes for detection.
 */
enum class ObjectClass {
    Car,
    Bicycle,
    Pedestrian,
    Unknown
};

inline std::string objectClassToString(ObjectClass cls) {
    switch (cls) {
        case ObjectClass::Car: return "Car";
        case ObjectClass::Bicycle: return "Bicycle";
        case ObjectClass::Pedestrian: return "Pedestrian";
        default: return "Unknown";
    }
}
inline ObjectClass stringToObjectClass(const std::string& str) {
    if (str == "Car") return ObjectClass::Car;
    if (str == "Bicycle") return ObjectClass::Bicycle;
    if (str == "Pedestrian") return ObjectClass::Pedestrian;
    return ObjectClass::Unknown;
}

/**
 * @brief Represents a point in a planned trajectory (for the ego vehicle).
 */
struct PlannedTrajPoint {
    glm::vec3 position;         ///< 3D position (x, y, z) of the trajectory point
    float heading;              ///< Heading angle (yaw) in degrees
    float long_vel;             ///< Longitudinal velocity at this point
    float lat_vel;              ///< Lateral velocity at this point
    float acc;                  ///< Acceleration at this point
    float time_from_start;      ///< Time from the start of the trajectory to reach this point
};

/**
 * @brief Represents the planned trajectory for the ego vehicle.
 */
class PlannedTraj {
public:    
    const std::vector<PlannedTrajPoint>& points() const { return points_; }
    std::vector<PlannedTrajPoint>& points() { return points_; }
private:
    std::vector<PlannedTrajPoint> points_; ///< Sequence of trajectory points
};

/**
 * @brief Represents a point in a predicted travel path.
 */
struct PredictedTravelPathPoint {
    glm::vec3 position;         ///< 3D position (x, y, z) of the predicted path point
    float heading;              ///< Heading angle (yaw) in degrees
};

/**
 * @brief Represents a predicted travel path for a detected object.
 */
class PredictedTravelPath {
public:
    float confidence() const { return confidence_; }
    float time_step() const { return time_step_; }
    const std::vector<PredictedTravelPathPoint>& path_points() const { return path_points_; }
    
    void set_confidence(float c) { confidence_ = c; }
    void set_time_step(float ts) { time_step_ = ts; }
    std::vector<PredictedTravelPathPoint>& path_points() { return path_points_; }

private:
    float confidence_;          ///< Confidence score of this predicted path
    float time_step_;           ///< Time step between consecutive points in this path
    std::vector<PredictedTravelPathPoint> path_points_; ///< Sequence of points in the predicted path
};

/**
 * @brief Represents a detected object in a frame.
 */
class PerceivedObject {
public:
    const std::string& id() const { return id_; }
    ObjectClass object_class() const { return objectClass_; }
    double probability() const { return probability_; }
    const glm::vec3& position() const { return position_; }
    float heading() const { return heading_; }
    const glm::vec3& velocity() const { return velocity_; }
    const std::vector<glm::vec2>& local_vertices() const { return local_vertices_; }
    const std::vector<PredictedTravelPath>& predicted_paths() const { return predicted_paths_; }
    
    void set_id(const std::string& id) { id_ = id; }
    void set_object_class(ObjectClass cls) { objectClass_ = cls; }
    void set_probability(double p) { probability_ = p; }
    void set_position(const glm::vec3& pos) { position_ = pos; }
    void set_heading(float h) { heading_ = h; }
    void set_velocity(const glm::vec3& vel) { velocity_ = vel; }
    std::vector<glm::vec2>& local_vertices() { return local_vertices_; }
    std::vector<PredictedTravelPath>& predicted_paths() { return predicted_paths_; }

private:
    std::string id_;            ///< Unique identifier for the object
    ObjectClass objectClass_;   ///< Class of the detected object
    double probability_;        ///< Detection confidence/probability [0, 1]
    glm::vec3 position_;        ///< 3D position (x, y, z) of the object
    float heading_;             ///< Heading angle (yaw) in degrees
    glm::vec3 velocity_;        ///< 3D velocity (vx, vy, vz) of the object
    std::vector<glm::vec2> local_vertices_; ///< Local vertices of the object's shape (for collision checking)
    std::vector<PredictedTravelPath> predicted_paths_; ///< Predicted travel paths for this object
};

/**
 * @brief Represents the runtime data for each planning cycle.
 */
class PlanningRuntimeData {
public:
    PlanningRuntimeData(const glm::vec2& ego_sz, const glm::vec2& ego_off)
        : ego_size(ego_sz), ego_center_offset(ego_off) {}
    PlanningRuntimeData(const glm::vec2& ego_sz, const glm::vec2& ego_off,
                        const std::vector<PerceivedObject>& perceived_objects,
                        const PlannedTraj& ego_planned_traj)
        : ego_size(ego_sz), ego_center_offset(ego_off), perceived_objects_(perceived_objects), ego_planned_traj_(ego_planned_traj) {}

    const glm::vec2 ego_size;
    const glm::vec2 ego_center_offset;
    
    const std::vector<PerceivedObject>& perceived_objects() const { return perceived_objects_; }
    const PlannedTraj& ego_planned_traj() const { return ego_planned_traj_; }
    
    std::vector<PerceivedObject>& perceived_objects() { return perceived_objects_; }
    PlannedTraj& ego_planned_traj() { return ego_planned_traj_; }

private:
    std::vector<PerceivedObject> perceived_objects_; ///< List of perceived objects in the current frame
    PlannedTraj ego_planned_traj_; ///< Planned trajectory for the ego vehicle
};

} // namespace fqltl

#endif // FQLTL_DATA_OBJECT_HPP
