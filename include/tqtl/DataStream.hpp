#ifndef TQTL_DATA_STREAM_HPP
#define TQTL_DATA_STREAM_HPP

#include "Frame.hpp"
#include <vector>
#include <stdexcept>
#include <ostream>

namespace tqtl {

/**
 * @brief Represents a data stream from a perception algorithm.
 * 
 * A data stream is a sequence of frames D(0), D(1), ..., D(n).
 * This class provides the main interface for accessing detection data
 * that will be evaluated against TQTL formulas.
 */
class DataStream {
public:
    DataStream() = default;

    /// Constructor with pre-allocated frame count
    explicit DataStream(size_t frameCount) {
        frames_.reserve(frameCount);
        for (size_t i = 0; i < frameCount; ++i) {
            frames_.emplace_back(i);
        }
    }

    size_t getFrameCount() const { return frames_.size(); }
    size_t length() const { return frames_.size(); }

    bool isEmpty() const { return frames_.empty(); }

    void addFrame(const Frame& frame) {
        frames_.push_back(frame);
    }
    /// Add a new frame to the stream (move semantics)
    void addFrame(Frame&& frame) {
        frames_.push_back(std::move(frame));
    }

    /// Add a new empty frame and return reference to it
    Frame& addFrame() {
        frames_.emplace_back(frames_.size());
        return frames_.back();
    }

    const Frame& getFrame(size_t index) const {
        if (index >= frames_.size()) {
            throw std::out_of_range("Frame index out of range: " + std::to_string(index));
        }
        return frames_[index];
    }
    /// Get a mutable frame by index
    Frame& getFrame(size_t index) {
        if (index >= frames_.size()) {
            throw std::out_of_range("Frame index out of range: " + std::to_string(index));
        }
        return frames_[index];
    }

    /// Operator [] for frame access (corresponds to D(i))
    const Frame& operator[](size_t index) const {
        return getFrame(index);
    }
    /// Operator [] for mutable frame access
    Frame& operator[](size_t index) {
        return getFrame(index);
    }

    /// Get set of object IDs at frame i (corresponds to SO(D(i)) in the paper)
    std::unordered_set<std::string> getObjectIds(size_t frameIndex) const {
        if (frameIndex >= frames_.size()) {
            return {};
        }
        return frames_[frameIndex].getObjectIds();
    }

    bool hasObject(size_t frameIndex, const std::string& objectId) const {
        if (frameIndex >= frames_.size()) {
            return false;
        }
        return frames_[frameIndex].hasObject(objectId);
    }

    /**
     * @brief Retrieve object data (corresponds to R(D(i), id) in the paper)
     * 
     * @param frameIndex The frame index
     * @param objectId The object ID
     * @return Pointer to the DataObject, or nullptr if not found
     */
    const DataObject* retrieve(size_t frameIndex, const std::string& objectId) const {
        if (frameIndex >= frames_.size()) {
            return nullptr;
        }
        return frames_[frameIndex].getObject(objectId);
    }

    /// Remove the oldest frame (first one) from the stream
    void removeOldestFrame() {
        if (!frames_.empty()) {
            frames_.erase(frames_.begin());
        }
    }

    const std::vector<Frame>& getFrames() const { return frames_; }
    void clear() { frames_.clear(); }

    // Iterator support
    auto begin() { return frames_.begin(); }
    auto end() { return frames_.end(); }
    auto begin() const { return frames_.begin(); }
    auto end() const { return frames_.end(); }
    auto cbegin() const { return frames_.cbegin(); }
    auto cend() const { return frames_.cend(); }

    std::string toString() const {
        std::string result = "DataStream (" + std::to_string(frames_.size()) + " frames):\n";
        for (const auto& frame : frames_) {
            result += "  " + frame.toString() + "\n";
        }
        return result;
    }
    friend std::ostream& operator<<(std::ostream& os, const DataStream& stream) {
        os << stream.toString();
        return os;
    }

    DataStream clone(size_t end, size_t start=0) const {
        if (start >= end || end > frames_.size()) {
            throw std::out_of_range("Invalid clone range: " + std::to_string(start) + " to " + std::to_string(end));
        }
        DataStream copy;
        copy.frames_ = std::vector<Frame>(frames_.begin() + start, frames_.begin() + end);
        return copy;
    }

private:
    std::vector<Frame> frames_;  ///< Sequence of frames
};

/**
 * @brief Builder class for creating DataStream instances.
 * 
 * Provides a fluent interface for constructing data streams.
 */
class DataStreamBuilder {
public:
    DataStreamBuilder() = default;

    DataStreamBuilder& newFrame() {
        stream_.addFrame();
        return *this;
    }

    /// Add an object to the current frame
    DataStreamBuilder& addObject(const std::string& id, ObjectClass cls, double probability, 
                                  const glm::vec3& position) {
        if (stream_.isEmpty()) {
            stream_.addFrame();
        }
        stream_.getFrame(stream_.getFrameCount() - 1)
               .addObject(DataObject(id, cls, probability, position));
        return *this;
    }
    DataStreamBuilder& addObject(const std::string& id, ObjectClass cls, double probability, 
                                  const glm::vec3& position, const glm::vec3& velocity, const BoundingBox& bbox) {
        if (stream_.isEmpty()) {
            stream_.addFrame();
        }
        auto obj = DataObject(id, cls, probability, position, velocity);
        obj.setBoundingBox(bbox);
        stream_.getFrame(stream_.getFrameCount() - 1)
               .addObject(obj);
        return *this;
    }

    /// Add an object to the current frame (string class)
    DataStreamBuilder& addObject(const std::string& id, const std::string& cls, double probability,
                                  const glm::vec3& position) {
        return addObject(id, stringToObjectClass(cls), probability, position);
    }

    DataStreamBuilder& addObject(const std::string& id, const std::string& cls, double probability,
                                  const glm::vec3& position, const glm::vec3& velocity, const BoundingBox& bbox) {
        return addObject(id, stringToObjectClass(cls), probability, position, velocity, bbox);
    }

    DataStream build() {
        return std::move(stream_);
    }

private:
    DataStream stream_;
};

} 
#endif // TQTL_DATA_STREAM_HPP
