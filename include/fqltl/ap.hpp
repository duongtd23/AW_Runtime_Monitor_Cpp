#pragma once
#include <string>
#include <map>
#include <functional>
#include <memory>
#include <sstream>
#include <cmath>
#include <stdexcept>

/// @brief Atomic Proposition (AP) classes for the fqltl (Finite-domain Quantified LTL) specification language.
///
/// Each AP class represents one kind of atomic predicate that can appear in
/// a quantified formula.  Before model-checking, every AP is grounded with
/// concrete variable bindings, producing:
///   1.  A unique propositional atom name suitable for Spot (e.g.
///       "collision__car1__path1").
///   2.  An evaluation function that decides the truth value of the AP
///       against a RuntimeState.
///
/// ## Qualified path IDs
/// When the grounder binds a *path* variable it uses a **qualified** ID of
/// the form  "obj_id__path_id"  (double underscore separator).  The same
/// separator is used in RuntimeState maps so that lookups are consistent.

namespace fqltl {

// ============================================================================
// RuntimeState — the data available at each trajectory step
// ============================================================================

/// All runtime data needed to evaluate APs at a single trajectory step.
///
/// Keys in the path maps use **qualified path IDs**:
///   qualified_path_id = obj_id + "__" + local_path_id
/// Example: "car1__path1", "ped2__path1"
struct RuntimeState {
    double time = 0.0;

    /// obj_id → existence probability
    std::map<std::string, double> object_existence_prob;

    /// qualified_path_id → confidence score
    std::map<std::string, double> path_confidence;

    /// qualified_path_id → true if ego collides with this path at this step
    std::map<std::string, bool>   path_collision;

    /// qualified_path_id → distance to ego at this step
    std::map<std::string, double> path_distance;

    /// ego speed at this step
    double ego_speed;

    /// ego acceleration at this step
    double ego_acceleration;
};

// ============================================================================
// VarBindings — maps formula variable names to concrete runtime IDs
// ============================================================================

/// Maps variable names (as written in the formula) to their concrete IDs.
/// Object variables map to an obj_id string.
/// Path variables map to a qualified_path_id string.
using VarBindings = std::map<std::string, std::string>;

// ============================================================================
// CompOp — comparison operators used in threshold APs
// ============================================================================

enum class CompOp { GT, GEQ, LT, LEQ, EQ };

inline const char* compOpStr(CompOp op) {
    switch (op) {
        case CompOp::GT:  return "gt";
        case CompOp::GEQ: return "ge";
        case CompOp::LT:  return "lt";
        case CompOp::LEQ: return "le";
        case CompOp::EQ:  return "eq";
        default:          return "?";
    }
}

inline const char* compOpSymbol(CompOp op) {
    switch (op) {
        case CompOp::GT:  return ">";
        case CompOp::GEQ: return ">=";
        case CompOp::LT:  return "<";
        case CompOp::LEQ: return "<=";
        case CompOp::EQ:  return "==";
        default:          return "?";
    }
}

inline bool evalCompOp(CompOp op, double lhs, double rhs) {
    switch (op) {
        case CompOp::GT:  return lhs >  rhs;
        case CompOp::GEQ: return lhs >= rhs;
        case CompOp::LT:  return lhs <  rhs;
        case CompOp::LEQ: return lhs <= rhs;
        case CompOp::EQ:  return std::abs(lhs - rhs) < 1e-9;
        default:          return false;
    }
}

// ============================================================================
// Helper — sanitize a double into a valid identifier fragment
// ============================================================================

/// Converts a double value to a string that is safe to embed inside a Spot
/// atom name (letters, digits, underscores only).
///
/// Examples:
///   0.5    → "0_5"
///   3.0    → "3"      (std::ostream drops trailing zeros)
///   0.1    → "0_1"
///   1e-05  → "1e_05"
///  -2.5    → "neg_2_5"
inline std::string sanitizeNumber(double v) {
    std::ostringstream ss;
    ss << v;
    std::string s = ss.str();

    std::string result;
    result.reserve(s.size() + 4);

    size_t i = 0;
    if (!s.empty() && s[0] == '-') {
        result += "neg_";
        i = 1;
    }
    for (; i < s.size(); ++i) {
        char c = s[i];
        if      (c == '.')             result += '_';
        else if (c == '-')             result += '_'; // minus in exponent
        else if (c == '+') { /* skip */ }             // plus in exponent
        else                           result += c;
    }
    return result;
}

// ============================================================================
// AP — abstract base class
// ============================================================================

/// Abstract base class for all atomic propositions.
class AP {
public:
    enum class Kind { Collision, ExistenceProb, Confidence, Time, Distance, EgoSpeed, EgoAcceleration };

    virtual ~AP() = default;

    virtual Kind kind() const = 0;

    /// Returns the propositional atom name after substituting variable
    /// bindings.  The name is a valid Spot identifier
    /// (lowercase letters, digits, underscores).
    virtual std::string groundedAtomName(const VarBindings& bindings) const = 0;

    /// Evaluates the AP against a runtime state using the supplied bindings.
    virtual bool evaluate(const RuntimeState& state,
                          const VarBindings& bindings) const = 0;

    /// Human-readable representation before grounding (variable names kept).
    virtual std::string toString() const = 0;
};

// ============================================================================
// CollisionAP — collision(obj_var, path_var)
// ============================================================================

/// Represents  collision(obj, path).
/// True when the ego trajectory collides with the predicted path at the
/// current time step.
///
/// Atom name: "collision__<qualified_path_id>"
/// Evaluation: state.path_collision[qualified_path_id]
class CollisionAP : public AP {
public:
    std::string obj_var;   ///< object variable name  (e.g. "obj")
    std::string path_var;  ///< path variable name    (e.g. "path")

    CollisionAP(std::string obj_var, std::string path_var)
        : obj_var(std::move(obj_var)), path_var(std::move(path_var)) {}

    Kind kind() const override { return Kind::Collision; }

    std::string groundedAtomName(const VarBindings& bindings) const override {
        auto pi = bindings.find(path_var);
        if (pi == bindings.end())
            throw std::runtime_error(
                "CollisionAP: unbound variable '" + path_var + "'");
        // The path variable is bound to a qualified_path_id (e.g.
        // "car1__path1"), which already encodes the object context.
        return "collision__" + pi->second;
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& bindings) const override {
        auto pi = bindings.find(path_var);
        if (pi == bindings.end())
            throw std::runtime_error(
                "CollisionAP: unbound variable '" + path_var + "'");
        auto it = state.path_collision.find(pi->second);
        return (it != state.path_collision.end()) && it->second;
    }

    std::string toString() const override {
        return "collision(" + obj_var + ", " + path_var + ")";
    }
};

// ============================================================================
// ExistenceProbAP — existenceProb(obj_var) op threshold
// ============================================================================

/// Represents  existenceProb(obj) op threshold.
/// Example: existenceProb(obj) > 0.5
///
/// Atom name: "existenceprob__<obj_id>__<op>__<threshold>"
/// Evaluation: state.object_existence_prob[obj_id] op threshold
class ExistenceProbAP : public AP {
public:
    std::string obj_var;   ///< object variable name
    CompOp      op;        ///< comparison operator
    double      threshold; ///< comparison threshold

    ExistenceProbAP(std::string obj_var, CompOp op, double threshold)
        : obj_var(std::move(obj_var)), op(op), threshold(threshold) {}

    Kind kind() const override { return Kind::ExistenceProb; }

    std::string groundedAtomName(const VarBindings& bindings) const override {
        auto oi = bindings.find(obj_var);
        if (oi == bindings.end())
            throw std::runtime_error(
                "ExistenceProbAP: unbound variable '" + obj_var + "'");
        return std::string("existenceprob__") + oi->second
             + "__" + compOpStr(op) + "__" + sanitizeNumber(threshold);
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& bindings) const override {
        auto oi = bindings.find(obj_var);
        if (oi == bindings.end())
            throw std::runtime_error(
                "ExistenceProbAP: unbound variable '" + obj_var + "'");
        auto it = state.object_existence_prob.find(oi->second);
        if (it == state.object_existence_prob.end()) return false;
        return evalCompOp(op, it->second, threshold);
    }

    std::string toString() const override {
        return "existenceProb(" + obj_var + ") "
             + compOpSymbol(op) + " " + std::to_string(threshold);
    }
};

// ============================================================================
// ConfidenceAP — confidence(path_var) op threshold
// ============================================================================

/// Represents  confidence(path) op threshold.
/// Example: confidence(path) >= 0.1
///
/// Atom name: "confidence__<qualified_path_id>__<op>__<threshold>"
/// Evaluation: state.path_confidence[qualified_path_id] op threshold
class ConfidenceAP : public AP {
public:
    std::string path_var;  ///< path variable name
    CompOp      op;
    double      threshold;

    ConfidenceAP(std::string path_var, CompOp op, double threshold)
        : path_var(std::move(path_var)), op(op), threshold(threshold) {}

    Kind kind() const override { return Kind::Confidence; }

    std::string groundedAtomName(const VarBindings& bindings) const override {
        auto pi = bindings.find(path_var);
        if (pi == bindings.end())
            throw std::runtime_error(
                "ConfidenceAP: unbound variable '" + path_var + "'");
        return "confidence__" + pi->second
             + "__" + compOpStr(op) + "__" + sanitizeNumber(threshold);
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& bindings) const override {
        auto pi = bindings.find(path_var);
        if (pi == bindings.end())
            throw std::runtime_error(
                "ConfidenceAP: unbound variable '" + path_var + "'");
        auto it = state.path_confidence.find(pi->second);
        if (it == state.path_confidence.end()) return false;
        return evalCompOp(op, it->second, threshold);
    }

    std::string toString() const override {
        return "confidence(" + path_var + ") "
             + compOpSymbol(op) + " " + std::to_string(threshold);
    }
};

// ============================================================================
// TimeAP — time op threshold
// ============================================================================

/// Represents  time op threshold.
/// Example: time <= 3.0
///
/// Atom name: "time__<op>__<threshold>"
/// Evaluation: state.time op threshold
class TimeAP : public AP {
public:
    CompOp op;
    double threshold;

    TimeAP(CompOp op, double threshold) : op(op), threshold(threshold) {}

    Kind kind() const override { return Kind::Time; }

    std::string groundedAtomName(const VarBindings& /*bindings*/) const override {
        return std::string("time__") + compOpStr(op)
             + "__" + sanitizeNumber(threshold);
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& /*bindings*/) const override {
        return evalCompOp(op, state.time, threshold);
    }

    std::string toString() const override {
        return std::string("time ") + compOpSymbol(op)
             + " " + std::to_string(threshold);
    }
};

// ============================================================================
// DistanceAP — distance(obj_var, path_var) op threshold
// ============================================================================

/// Represents  distance(obj, path) op threshold.
/// Example: distance(obj, path) < 5.0: The distance between the ego (along the planned trajectory) and the object $obj along the predicted path $path is less than 5 meters at the current time step.
///
/// Atom name: "distance__<qualified_path_id>__<op>__<threshold>"
/// Evaluation: state.path_distance[qualified_path_id] op threshold
class DistanceAP : public AP {
public:
    std::string obj_var;   ///< object variable name  (e.g. "obj")
    std::string path_var;  ///< path variable name
    CompOp      op;
    double      threshold;

    DistanceAP(std::string obj_var, std::string path_var, CompOp op, double threshold)
        : obj_var(std::move(obj_var)), path_var(std::move(path_var)),
          op(op), threshold(threshold) {}

    Kind kind() const override { return Kind::Distance; }

    std::string groundedAtomName(const VarBindings& bindings) const override {
        auto pi = bindings.find(path_var);
        if (pi == bindings.end())
            throw std::runtime_error(
                "DistanceAP: unbound variable '" + path_var + "'");
        return "distance__" + pi->second
             + "__" + compOpStr(op) + "__" + sanitizeNumber(threshold);
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& bindings) const override {
        auto pi = bindings.find(path_var);
        if (pi == bindings.end())
            throw std::runtime_error(
                "DistanceAP: unbound variable '" + path_var + "'");
        auto it = state.path_distance.find(pi->second);
        if (it == state.path_distance.end()) return false;
        return evalCompOp(op, it->second, threshold);
    }

    std::string toString() const override {
        return "distance(" + obj_var + ", " + path_var + ") "
             + compOpSymbol(op) + " " + std::to_string(threshold);
    }
};

// ============================================================================
// EgoSpeedAP — speed op threshold
// ============================================================================

/// Represents  speed op threshold.
/// Example: speed <= 4.0: The ego speed along the planned trajectory is less than or equal to 4 m/s at the current time step.
///
/// Atom name: "speed__<op>__<threshold>"
/// Evaluation: state.ego_speed op threshold
class EgoSpeedAP : public AP {
public:
    CompOp op;
    double threshold;

    EgoSpeedAP(CompOp op, double threshold) : op(op), threshold(threshold) {}

    Kind kind() const override { return Kind::EgoSpeed; }

    std::string groundedAtomName(const VarBindings& /*bindings*/) const override {
        return std::string("speed__") + compOpStr(op)
             + "__" + sanitizeNumber(threshold);
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& /*bindings*/) const override {
        return evalCompOp(op, state.ego_speed, threshold);
    }

    std::string toString() const override {
        return std::string("speed ") + compOpSymbol(op)
             + " " + std::to_string(threshold);
    }
};

// ============================================================================
// EgoAccelerationAP — acceleration op threshold
// ============================================================================

/// Represents  acceleration op threshold.
/// Example: acceleration >= 2.0: The ego acceleration along the planned trajectory is greater than or equal to 2 m/s^2 at the current time step.
///
/// Atom name: "acceleration__<op>__<threshold>"
/// Evaluation: state.ego_acceleration op threshold
class EgoAccelerationAP : public AP {
public:
    CompOp op;
    double threshold;

    EgoAccelerationAP(CompOp op, double threshold) : op(op), threshold(threshold) {}

    Kind kind() const override { return Kind::EgoAcceleration; }

    std::string groundedAtomName(const VarBindings& /*bindings*/) const override {
        return std::string("acceleration__") + compOpStr(op)
             + "__" + sanitizeNumber(threshold);
    }

    bool evaluate(const RuntimeState& state,
                  const VarBindings& /*bindings*/) const override {
        return evalCompOp(op, state.ego_acceleration, threshold);
    }

    std::string toString() const override {
        return std::string("acceleration ") + compOpSymbol(op)
             + " " + std::to_string(threshold);
    }
};

} // namespace fqltl
