#ifndef TQTL_FRAME_HPP
#define TQTL_FRAME_HPP

#include "DataObject.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <ostream>

namespace tqtl {

/**
 * @brief Represents a single frame in a data stream.
 * 
 * A frame contains a set of detected objects at a specific timestamp.
 * This corresponds to D(i) in the paper.
 */
class Frame {
public:
    Frame() : timestamp_(0.0) {}
    explicit Frame(double timestamp) : timestamp_(timestamp) {}

    double getTimestamp() const { return timestamp_; }
    void setTimestamp(double timestamp) { timestamp_ = timestamp; }

    void addObject(const DataObject& obj) {
        objects_.push_back(obj);
        objectIndex_[obj.getId()] = objects_.size() - 1;
    }
    /// Add an object to the frame (move semantics)
    void addObject(DataObject&& obj) {
        std::string id = obj.getId();
        objects_.push_back(std::move(obj));
        objectIndex_[id] = objects_.size() - 1;
    }

    size_t getObjectCount() const { return objects_.size(); }
    const std::vector<DataObject>& getObjects() const { return objects_; }
    bool isEmpty() const { return objects_.empty(); }
    bool hasObject(const std::string& id) const {
        return objectIndex_.find(id) != objectIndex_.end();
    }

    /// Get an object by ID (corresponds to R(D(i), id) in the paper)
    /// Returns nullptr if the object is not found
    const DataObject* getObject(std::string id) const {
        auto it = objectIndex_.find(id);
        if (it != objectIndex_.end()) {
            return &objects_[it->second];
        }
        return nullptr;
    }

    /// Get the set of object IDs (corresponds to SO(D(i)) in the paper)
    std::unordered_set<std::string> getObjectIds() const {
        std::unordered_set<std::string> ids;
        for (const auto& obj : objects_) {
            ids.insert(obj.getId());
        }
        return ids;
    }

    /// Get object IDs as a vector (for iteration)
    std::vector<std::string> getObjectIdVector() const {
        std::vector<std::string> ids;
        ids.reserve(objects_.size());
        for (const auto& obj : objects_) {
            ids.push_back(obj.getId());
        }
        return ids;
    }

    /// Remove an object by ID
    bool removeObject(const std::string& id) {
        auto it = objectIndex_.find(id);
        if (it == objectIndex_.end()) {
            return false;
        }
        
        size_t index = it->second;
        
        // If not the last element, swap with last
        if (index < objects_.size() - 1) {
            std::string lastId = objects_.back().getId();
            std::swap(objects_[index], objects_.back());
            objectIndex_[lastId] = index;
        }
        
        objects_.pop_back();
        objectIndex_.erase(it);
        return true;
    }

    void clear() {
        objects_.clear();
        objectIndex_.clear();
    }

    std::string toString() const {
        std::string result = "D(" + std::to_string(timestamp_) + ") = {";
        for (size_t i = 0; i < objects_.size(); ++i) {
            if (i > 0) result += ", ";
            result += objects_[i].toString();
        }
        result += "}";
        return result;
    }

    friend std::ostream& operator<<(std::ostream& os, const Frame& frame) {
        os << frame.toString();
        return os;
    }

    const EgoObject& getEgoObject() const { return egoObject_; }
    void setEgoObject(const EgoObject& ego) { egoObject_ = ego; }
    void setEgoObject(EgoObject&& ego) { egoObject_ = std::move(ego); }

private:
    double timestamp_;                              ///< Frame timestamp/index
    std::vector<DataObject> objects_;               ///< Objects in this frame
    std::unordered_map<std::string, size_t> objectIndex_;   ///< Map from object ID to index
    EgoObject egoObject_;                            ///< Ego vehicle information
};

}

#endif // TQTL_FRAME_HPP
