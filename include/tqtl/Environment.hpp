#ifndef TQTL_ENVIRONMENT_HPP
#define TQTL_ENVIRONMENT_HPP

#include <unordered_map>
#include <string>
#include <stdexcept>
#include <ostream>
#include <sstream>

namespace tqtl {

/**
 * @brief Environment for TQTL formula evaluation.
 * 
 * The environment (ε) maps:
 * - Time variables (Vt) -> frame indices (N)
 * - Object variables (Vo) -> object IDs (N)
 * 
 * This corresponds to ε : Vt ∪ Vo -> N in the paper.
 * 
 * The environment supports the [x <== a] operation, which creates a new
 * environment identical to the current one except that variable x is
 * bound to value a.
 */
class Environment {
public:
    Environment() = default;
    Environment(const Environment& other) = default;
    /// Move constructor
    Environment(Environment&& other) noexcept = default;

    /// Copy assignment
    Environment& operator=(const Environment& other) = default;
    /// Move assignment
    Environment& operator=(Environment&& other) noexcept = default;

    /**
     * @brief Bind a time variable to a frame index.
     * 
     * @param varName The name of the time variable (e.g., "x", "y")
     * @param frameIndex The frame index to bind
     * @return Reference to this environment for chaining
     */
    Environment& bindTime(const std::string& varName, size_t frameIndex) {
        timeBindings_[varName] = frameIndex;
        return *this;
    }

    /**
     * @brief Bind an object variable to an object ID.
     * 
     * @param varName The name of the object variable (e.g., "id1", "id2")
     * @param objectId The object ID to bind
     * @return Reference to this environment for chaining
     */
    Environment& bindObject(const std::string& varName, const std::string& objectId) {
        objectBindings_[varName] = objectId;
        return *this;
    }

    /**
     * @brief Get the frame index bound to a time variable.
     * 
     * Corresponds to ε(x) where x ∈ Vt
     * 
     * @param varName The time variable name
     * @return The frame index
     * @throws std::out_of_range if variable is not bound
     */
    size_t getTime(const std::string& varName) const {
        auto it = timeBindings_.find(varName);
        if (it == timeBindings_.end()) {
            throw std::out_of_range("Time variable not bound: " + varName);
        }
        return it->second;
    }

    /**
     * @brief Get the object ID bound to an object variable.
     * 
     * Corresponds to ε(id) where id ∈ Vo
     * 
     * @param varName The object variable name
     * @return The object ID
     * @throws std::out_of_range if variable is not bound
     */
    std::string getObject(const std::string& varName) const {
        auto it = objectBindings_.find(varName);
        if (it == objectBindings_.end()) {
            throw std::out_of_range("Object variable not bound: " + varName);
        }
        return it->second;
    }

    /**
     * @brief Check if a time variable is bound.
     */
    bool hasTime(const std::string& varName) const {
        return timeBindings_.find(varName) != timeBindings_.end();
    }

    /**
     * @brief Check if an object variable is bound.
     */
    bool hasObject(const std::string& varName) const {
        return objectBindings_.find(varName) != objectBindings_.end();
    }

    /**
     * @brief Create a new environment with time variable bound to a new value.
     * 
     * Corresponds to ε[x <== i] in the paper.
     * Creates a copy of this environment with the specified binding updated.
     * 
     * @param varName The time variable name
     * @param frameIndex The new frame index
     * @return A new Environment with the updated binding
     */
    Environment withTime(const std::string& varName, size_t frameIndex) const {
        Environment newEnv(*this);
        newEnv.bindTime(varName, frameIndex);
        return newEnv;
    }

    /** 
     * @brief Create a new environment with object variable bound to a new value.
     * 
     * Corresponds to ε[id <== k] in the paper.
     * Creates a copy of this environment with the specified binding updated.
     * 
     * @param varName The object variable name
     * @param objectId The new object ID
     * @return A new Environment with the updated binding
     */
    Environment withObject(const std::string& varName, const std::string& objectId) const {
        Environment newEnv(*this);
        newEnv.bindObject(varName, objectId);
        return newEnv;
    }

    /**
     * @brief Remove a time variable binding.
     */
    void unbindTime(const std::string& varName) {
        timeBindings_.erase(varName);
    }

    /**
     * @brief Remove an object variable binding.
     */
    void unbindObject(const std::string& varName) {
        objectBindings_.erase(varName);
    }

    /**
     * @brief Clear all bindings.
     */
    void clear() {
        timeBindings_.clear();
        objectBindings_.clear();
    }

    /**
     * @brief Get the number of time variable bindings.
     */
    size_t timeBindingCount() const {
        return timeBindings_.size();
    }

    /**
     * @brief Get the number of object variable bindings.
     */
    size_t objectBindingCount() const {
        return objectBindings_.size();
    }

    /**
     * @brief Check if the environment is empty.
     */
    bool isEmpty() const {
        return timeBindings_.empty() && objectBindings_.empty();
    }

    /**
     * @brief Get all time bindings (for iteration/debugging).
     */
    const std::unordered_map<std::string, size_t>& getTimeBindings() const {
        return timeBindings_;
    }

    /**
     * @brief Get all object bindings (for iteration/debugging).
     */
    const std::unordered_map<std::string, std::string>& getObjectBindings() const {
        return objectBindings_;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Environment{";
        
        bool first = true;
        oss << "time:[";
        for (const auto& [name, value] : timeBindings_) {
            if (!first) oss << ", ";
            oss << name << "=" << value;
            first = false;
        }
        oss << "]";
        
        first = true;
        oss << ", obj:[";
        for (const auto& [name, value] : objectBindings_) {
            if (!first) oss << ", ";
            oss << name << "=" << value;
            first = false;
        }
        oss << "]";
        
        oss << "}";
        return oss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const Environment& env) {
        os << env.toString();
        return os;
    }

    bool operator==(const Environment& other) const {
        return timeBindings_ == other.timeBindings_ && 
               objectBindings_ == other.objectBindings_;
    }
    bool operator!=(const Environment& other) const {
        return !(*this == other);
    }

private:
    std::unordered_map<std::string, size_t> timeBindings_;           ///< Time variables -> frame indices
    std::unordered_map<std::string, std::string> objectBindings_;    ///< Object variables -> object IDs
};

/**
 * @brief Builder class for creating Environment instances.
 * 
 * Provides a fluent interface for constructing environments.
 */
class EnvironmentBuilder {
public:
    EnvironmentBuilder() = default;

    /**
     * @brief Add a time variable binding.
     */
    EnvironmentBuilder& time(const std::string& varName, size_t frameIndex) {
        env_.bindTime(varName, frameIndex);
        return *this;
    }

    /**
     * @brief Add an object variable binding.
     */
    EnvironmentBuilder& object(const std::string& varName, const std::string& objectId) {
        env_.bindObject(varName, objectId);
        return *this;
    }

    /**
     * @brief Build and return the environment.
     */
    Environment build() {
        return std::move(env_);
    }

private:
    Environment env_;
};

}

#endif // TQTL_ENVIRONMENT_HPP
