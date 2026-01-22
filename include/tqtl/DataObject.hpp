#ifndef TQTL_DATA_OBJECT_HPP
#define TQTL_DATA_OBJECT_HPP

#include "BoundingBox.hpp"
#include <string>
#include <ostream>
#include <unordered_map>
#include <variant>
#include <optional>
#include <glm/glm.hpp>  // for vector computation
#include <glm/gtx/string_cast.hpp>

namespace tqtl {

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
 * @brief Represents a detected object in a frame.
 * 
 * Each data object has a unique ID, class, and probability.
 * Additionally, it contains 3D position and velocity information, bounding box, etc.
 */
class DataObject {
public:
    using AttributeValue = std::variant<int, double, std::string, bool>;

    DataObject() : id_("0"), objectClass_(ObjectClass::Unknown), probability_(0.0) {}

    DataObject(const std::string& id, ObjectClass objectClass, double probability, const glm::vec3& position)
        : id_(id), objectClass_(objectClass), probability_(probability), position_(position) {}
    DataObject(const std::string& id, ObjectClass objectClass, double probability, const glm::vec3& position, const glm::vec3& velocity)
        : id_(id), objectClass_(objectClass), probability_(probability), position_(position), velocity_(velocity) {}

    DataObject(const std::string& id, const std::string& objectClass, double probability, const glm::vec3& position)
        : id_(id), objectClass_(stringToObjectClass(objectClass)), 
          probability_(probability), position_(position) {}
    DataObject(const std::string& id, const std::string& objectClass, double probability, const glm::vec3& position, const glm::vec3& velocity)
        : id_(id), objectClass_(stringToObjectClass(objectClass)), 
          probability_(probability), position_(position), velocity_(velocity) {}

    std::string getId() const { return id_; }
    ObjectClass getObjectClass() const { return objectClass_; }
    std::string getClassName() const { return objectClassToString(objectClass_); }
    double getProbability() const { return probability_; }
    const glm::vec3& getPosition() const { return position_; }
    const glm::vec3& getVelocity() const { return velocity_; }
    double getSpeed() const { return glm::length(velocity_); }
    const BoundingBox& getBoundingBox() const { return boundingBox_; }

    void setId(const std::string& id) { id_ = id; }
    void setObjectClass(ObjectClass cls) { objectClass_ = cls; }
    void setObjectClass(const std::string& cls) { objectClass_ = stringToObjectClass(cls); }
    void setProbability(double prob) { probability_ = prob; }
    void setPosition(const glm::vec3& pos) { position_ = pos; }
    void setVelocity(const glm::vec3& vel) { velocity_ = vel; }
    void setBoundingBox(const BoundingBox& bbox) { boundingBox_ = bbox; }

    void setAttribute(const std::string& name, const AttributeValue& value) {
        attributes_[name] = value;
    }
    std::optional<AttributeValue> getAttribute(const std::string& name) const {
        auto it = attributes_.find(name);
        if (it != attributes_.end()) {
            return it->second;
        }
        return std::nullopt;
    }
    bool hasAttribute(const std::string& name) const {
        return attributes_.find(name) != attributes_.end();
    }

    bool operator==(const DataObject& other) const {
        return id_ == other.id_;
    }
    bool operator!=(const DataObject& other) const {
        return !(*this == other);
    }

    std::string toString() const {
        return "(" + id_ + ", \"" + getClassName() + "\", " +
               std::to_string(probability_) + ", " + glm::to_string(position_) + ")";
    }
    friend std::ostream& operator<<(std::ostream& os, const DataObject& obj) {
        os << obj.toString();
        return os;
    }

private:
    std::string id_;            ///< Unique identifier for the object
    ObjectClass objectClass_;   ///< Class of the detected object
    double probability_;        ///< Detection confidence/probability [0, 1]
    glm::vec3 position_;        ///< 3D position (x, y, z) of the object
    glm::vec3 velocity_;        ///< 3D velocity (vx, vy, vz) of the object
    BoundingBox boundingBox_;   ///< Bounding box coordinates
    std::unordered_map<std::string, AttributeValue> attributes_; ///< Custom attributes
};

}

#endif // TQTL_DATA_OBJECT_HPP
