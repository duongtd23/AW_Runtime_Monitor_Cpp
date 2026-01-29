#ifndef TQTL_PREDICATE_HPP
#define TQTL_PREDICATE_HPP

#include "Formula.hpp"
#include "QualityValue.hpp"
#include "DataStream.hpp"
#include "Environment.hpp"
#include <string>
#include <functional>
#include <memory>
#include <cmath>
#include <optional>
#include <glm/gtx/vector_angle.hpp>  // for glm::angle

namespace tqtl {

// ============================================================================
// Comparison Operators
// ============================================================================

/**
 * @brief Comparison operators for predicates.
 */
enum class ComparisonOp {
    LT,   // <
    LE,   // <=
    GT,   // >
    GE,   // >=
    EQ,   // =
    NE    // !=
};
inline std::string comparisonOpToString(ComparisonOp op) {
    switch (op) {
        case ComparisonOp::LT: return "<";
        case ComparisonOp::LE: return "≤";
        case ComparisonOp::GT: return ">";
        case ComparisonOp::GE: return "≥";
        case ComparisonOp::EQ: return "=";
        case ComparisonOp::NE: return "≠";
        default: return "?";
    }
}

inline ComparisonOp stringToComparisonOp(const std::string& str) {
    if (str == "<") return ComparisonOp::LT;
    if (str == "<=" || str == "≤") return ComparisonOp::LE;
    if (str == ">") return ComparisonOp::GT;
    if (str == ">=" || str == "≥") return ComparisonOp::GE;
    if (str == "=" || str == "==") return ComparisonOp::EQ;
    if (str == "!=" || str == "≠") return ComparisonOp::NE;
    throw std::invalid_argument("Unknown comparison operator: " + str);
}

/**
 * @brief Compute robustness value for a comparison.
 * 
 * For value `v` compared against threshold `t`:
 * - GT (>):  robustness = v - t  (positive if v > t)
 * - GE (>=): robustness = v - t  (non-negative if v >= t)
 * - LT (<):  robustness = t - v  (positive if v < t)
 * - LE (<=): robustness = t - v  (non-negative if v <= t)
 * - EQ (=):  robustness = -|v - t| (zero if v == t, negative otherwise)
 * - NE (!=): robustness = |v - t|  (positive if v != t, zero otherwise)
 */
inline QualityValue computeRobustness(double value, double threshold, ComparisonOp op) {
    switch (op) {
        case ComparisonOp::GT:
        case ComparisonOp::GE:
            return QualityValue(value - threshold);
        case ComparisonOp::LT:
        case ComparisonOp::LE:
            return QualityValue(threshold - value);
        case ComparisonOp::EQ:
            return QualityValue(-std::abs(value - threshold));
        case ComparisonOp::NE:
            return QualityValue(std::abs(value - threshold));
        default:
            return QualityValue::NEG_INF;
    }
}

/**
 * @brief Check if a comparison is satisfied (for Boolean predicates like class equality).
 */
inline bool checkComparison(double value, double threshold, ComparisonOp op) {
    switch (op) {
        case ComparisonOp::GT: return value > threshold;
        case ComparisonOp::GE: return value >= threshold;
        case ComparisonOp::LT: return value < threshold;
        case ComparisonOp::LE: return value <= threshold;
        case ComparisonOp::EQ: return value == threshold;
        case ComparisonOp::NE: return value != threshold;
        default: return false;
    }
}

inline std::optional<size_t> computeRefFrame(const FrameExpr& frameExpr, size_t baseFrame, const DataStream& stream) {
    // Compute the reference frame with offset
    size_t frame;
    if (frameExpr.offset >= 0) {
        frame = baseFrame + static_cast<size_t>(frameExpr.offset);
    } else {
        // Handle negative offset - check for underflow
        size_t absOffset = static_cast<size_t>(-frameExpr.offset);
        if (baseFrame < absOffset) {
            // Frame would be negative - return +∞ (trivially satisfied)
            return std::nullopt;
        }
        frame = baseFrame - absOffset;
    }
    
    // Check if frame is within stream bounds
    // If out of bounds, return +∞ (property trivially satisfied at end of stream)
    if (frame >= stream.length()) {
        return std::nullopt;
    }
    return frame;
}

/**
 * @brief Type alias for scoring functions.
 * 
 * A scoring function takes a DataStream, frame index, and Environment,
 * and returns a QualityValue.
 */
using ScoringFunction = std::function<QualityValue(const DataStream&, size_t, const Environment&)>;

/**
 * @brief Concrete predicate with a custom scoring function.
 * 
 * This allows users to define arbitrary predicates with custom evaluation logic.
 */
class CustomPredicate : public PredicateFormula {
public:
    CustomPredicate(const std::string& name, ScoringFunction scoringFn)
        : name_(name), scoringFn_(std::move(scoringFn)) {}

    std::string toString() const override { return name_; }
    
    void accept(FormulaVisitor& visitor) const override { 
        visitor.visit(*this); 
    }
    
    FormulaPtr clone() const override {
        return std::make_shared<CustomPredicate>(name_, scoringFn_);
    }

    const std::string& getName() const { return name_; }

    /// Evaluate the predicate
    QualityValue evaluate(const DataStream& stream, size_t frameIndex, const Environment& env) const {
        return scoringFn_(stream, frameIndex, env);
    }

private:
    std::string name_;
    ScoringFunction scoringFn_;
};

/**
 * @brief Predicate for checking object class equality: C(timeVar, objectVar) = class
 * 
 * Returns +∞ if the object's class matches, -∞ otherwise.
 */
class ClassEqualsPredicate : public PredicateFormula {
public:
    ClassEqualsPredicate(const FrameExpr& frameExpr, const std::string& objectVar, ObjectClass expectedClass)
        : frameExpr_(frameExpr), objectVar_(objectVar), expectedClass_(expectedClass) {}

    std::string toString() const override {
        return "C(" + frameExpr_.toString() + ", " + objectVar_ + ") = " + objectClassToString(expectedClass_);
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<ClassEqualsPredicate>(frameExpr_, objectVar_, expectedClass_);
    }

    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    const std::string& getObjectVar() const { return objectVar_; }
    ObjectClass getExpectedClass() const { return expectedClass_; }

    /// Evaluate the predicate
    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        const DataObject* obj = stream.retrieve(frame, objId);
        if (!obj) {
            return QualityValue::NEG_INF;  // Object not found
        }

        if (obj->getObjectClass() == expectedClass_) {
            return QualityValue::POS_INF;
        }
        return QualityValue::NEG_INF;
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
    ObjectClass expectedClass_;
};

/**
 * @brief Predicate for checking object class inequality: C(timeVar, objectVar) != class
 */
class ClassNotEqualsPredicate : public PredicateFormula {
public:
    ClassNotEqualsPredicate(const FrameExpr& frameExpr, const std::string& objectVar, ObjectClass excludedClass)
        : frameExpr_(frameExpr), objectVar_(objectVar), excludedClass_(excludedClass) {}

    std::string toString() const override {
        return "C(" + frameExpr_.toString() + ", " + objectVar_ + ") ≠ " + objectClassToString(excludedClass_);
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<ClassNotEqualsPredicate>(frameExpr_, objectVar_, excludedClass_);
    }

    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    const std::string& getObjectVar() const { return objectVar_; }
    ObjectClass getExcludedClass() const { return excludedClass_; }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        const DataObject* obj = stream.retrieve(frame, objId);
        if (!obj) {
            return QualityValue::NEG_INF;
        }

        if (obj->getObjectClass() != excludedClass_) {
            return QualityValue::POS_INF;
        }
        return QualityValue::NEG_INF;
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
    ObjectClass excludedClass_;
};

// ============================================================================
// Probability Predicate (unified for <, <=, >, >=)
// ============================================================================

/**
 * @brief Predicate for probability comparison: P(timeVar, objectVar) op threshold
 * 
 * Returns robustness value based on (probability - threshold) or (threshold - probability).
 */
class ProbabilityPredicate : public PredicateFormula {
public:
    ProbabilityPredicate(const FrameExpr& frameExpr, const std::string& objectVar,
                         ComparisonOp op, double threshold)
        : frameExpr_(frameExpr), objectVar_(objectVar), op_(op), threshold_(threshold) {}

    std::string toString() const override {
        return "P(" + frameExpr_.toString() + ", " + objectVar_ + ") " + 
               comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<ProbabilityPredicate>(frameExpr_, objectVar_, op_, threshold_);
    }

    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    const std::string& getObjectVar() const { return objectVar_; }
    ComparisonOp getOp() const { return op_; }
    double getThreshold() const { return threshold_; }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        const DataObject* obj = stream.retrieve(frame, objId);
        if (!obj) {
            return QualityValue::NEG_INF;
        }
        return computeRobustness(obj->getProbability(), threshold_, op_);
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
    ComparisonOp op_;
    double threshold_;
};

/**
 * @brief Predicate for IoU (Intersection over Union): IoU(t1, t2, id1, id2) > threshold
 */
class IoUPredicate : public PredicateFormula {
public:
    IoUPredicate(const FrameExpr& frameExpr1, const std::string& objectVar1,
                 const FrameExpr& frameExpr2, const std::string& objectVar2,
                 ComparisonOp op, double threshold)
        : frameExpr1_(frameExpr1), frameExpr2_(frameExpr2),
          objectVar1_(objectVar1), objectVar2_(objectVar2),
          op_(op), threshold_(threshold) {}

    std::string toString() const override {
        return "IoU(" + frameExpr1_.toString() + ", " + objectVar1_ + ", " + 
            frameExpr2_.toString() + ", " + objectVar2_ + ") " + comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<IoUPredicate>(
            frameExpr1_, objectVar1_, frameExpr2_, objectVar2_, op_, threshold_);
    }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame1 = env.getTime(frameExpr1_.varName);
        size_t baseFrame2 = env.getTime(frameExpr2_.varName);
        auto frameOrNull1 = computeRefFrame(frameExpr1_, baseFrame1, stream);
        auto frameOrNull2 = computeRefFrame(frameExpr2_, baseFrame2, stream);

        if (!frameOrNull1.has_value() || !frameOrNull2.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame1 = frameOrNull1.value();
        size_t frame2 = frameOrNull2.value();

        std::string objId1 = env.getObject(objectVar1_);
        std::string objId2 = env.getObject(objectVar2_);

        const DataObject* obj1 = stream.retrieve(frame1, objId1);
        const DataObject* obj2 = stream.retrieve(frame2, objId2);

        if (!obj1 || !obj2) {
            return QualityValue::NEG_INF;
        }

        double iou = obj1->getBoundingBox().iou(obj2->getBoundingBox());
        return computeRobustness(iou, threshold_, op_);
    }

private:
    FrameExpr frameExpr1_;
    FrameExpr frameExpr2_;
    std::string objectVar1_;
    std::string objectVar2_;
    ComparisonOp op_;
    double threshold_;
};

/**
 * @brief Predicate to check if an object exists in the frame
 */
class ObjectExistsPredicate : public PredicateFormula {
public:
    ObjectExistsPredicate(const FrameExpr& frameExpr, const std::string& objectVar)
        : frameExpr_(frameExpr), objectVar_(objectVar) {}

    std::string toString() const override {
        return "exists(" + objectVar_ + "@" + frameExpr_.toString() + ")";
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<ObjectExistsPredicate>(frameExpr_, objectVar_);
    }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        if (stream.hasObject(frame, objId)) {
            return QualityValue::POS_INF;
        }
        return QualityValue::NEG_INF;
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
};

// ============================================================================
// Distance Predicate (cross-object, cross-frame)
// ============================================================================

/**
 * @brief Predicate for position distance between two objects (potentially at different frames).
 * 
 * Syntax: dist_pos(t1, id1, t2, id2) op threshold
 * 
 * This measures the Euclidean distance between the positions of object id1 at frame t1
 * and object id2 at frame t2. The objects may have different IDs (useful when tracking fails).
 */
class DistancePredicate : public PredicateFormula {
public:
    DistancePredicate(const FrameExpr& frameExpr1, const std::string& objectVar1,
                      const FrameExpr& frameExpr2, const std::string& objectVar2,
                      ComparisonOp op, double threshold)
        : frameExpr1_(frameExpr1), objectVar1_(objectVar1),
          frameExpr2_(frameExpr2), objectVar2_(objectVar2),
          op_(op), threshold_(threshold) {}

    std::string toString() const override {
        if (objectVar1_ == objectVar2_) {
            return "dist(" + frameExpr1_.toString() + ", " + frameExpr2_.toString() + ", " +
                   objectVar1_ + ") " + comparisonOpToString(op_) + " " + std::to_string(threshold_);
        }
        return "dist_pos(" + frameExpr1_.toString() + ", " + objectVar1_ + ", " +
               frameExpr2_.toString() + ", " + objectVar2_ + ") " +
               comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }
    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<DistancePredicate>(
            frameExpr1_, objectVar1_, frameExpr2_, objectVar2_, op_, threshold_);
    }

    const FrameExpr& getFrameExpr1() const { return frameExpr1_; }
    const FrameExpr& getFrameExpr2() const { return frameExpr2_; }
    const std::string& getObjectVar1() const { return objectVar1_; }
    const std::string& getObjectVar2() const { return objectVar2_; }
    ComparisonOp getOp() const { return op_; }
    double getThreshold() const { return threshold_; }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame1 = env.getTime(frameExpr1_.varName);
        size_t baseFrame2 = env.getTime(frameExpr2_.varName);
        auto frameOrNull1 = computeRefFrame(frameExpr1_, baseFrame1, stream);
        auto frameOrNull2 = computeRefFrame(frameExpr2_, baseFrame2, stream);

        if (!frameOrNull1.has_value() || !frameOrNull2.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame1 = frameOrNull1.value();
        size_t frame2 = frameOrNull2.value();

        std::string objId1 = env.getObject(objectVar1_);
        std::string objId2 = env.getObject(objectVar2_);

        const DataObject* obj1 = stream.retrieve(frame1, objId1);
        const DataObject* obj2 = stream.retrieve(frame2, objId2);

        if (!obj1 || !obj2) {
            return QualityValue::NEG_INF;
        }

        // Compute Euclidean distance between positions
        glm::vec3 diff = obj1->getPosition() - obj2->getPosition();
        double distance = glm::length(diff);
        
        return computeRobustness(distance, threshold_, op_);
    }

private:
    FrameExpr frameExpr1_;
    std::string objectVar1_;
    FrameExpr frameExpr2_;
    std::string objectVar2_;
    ComparisonOp op_;
    double threshold_;
};

/**
 * @brief Predicate for distance between object id1 at time t1 and
 *    the estimated position of object id2 at time t2_est.
 * Object id2 does not exist in t2_est, so its position is estimated based on its
 * position and velocity at frame t2_ref.
 *
 * Syntax: dist_est(t1, id1, t2_ref, id2, t2_est) op threshold
 * 
 * This measures the Euclidean distance between the position of object id1 at frame t1
 * and the estimated position of object id2 at frame t2_est, where the estimation is based
 * on the position and velocity of object id2 at frame t2_ref.
 */
class DistanceEstimationPredicate : public PredicateFormula {
public:
    DistanceEstimationPredicate(const FrameExpr& frameExpr1, const std::string& objectVar1,
                                const FrameExpr& frameRefExpr2, const std::string& objectVar2,
                                const FrameExpr& frameEstimationExpr2, ComparisonOp op, double threshold)
        : frameExpr1_(frameExpr1), objectVar1_(objectVar1),
          frameRefExpr2_(frameRefExpr2), objectVar2_(objectVar2),
          frameEstimationExpr2_(frameEstimationExpr2), op_(op), threshold_(threshold) {}
    std::string toString() const override {
        return "dist_est(" + frameExpr1_.toString() + ", " + objectVar1_ + ", " +
               frameRefExpr2_.toString() + ", " + objectVar2_ + ", " +
               frameEstimationExpr2_.toString() + ") " +
               comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }
    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }
    FormulaPtr clone() const override {
        return std::make_shared<DistanceEstimationPredicate>(
            frameExpr1_, objectVar1_,
            frameRefExpr2_, objectVar2_,
            frameEstimationExpr2_, op_, threshold_);
    }
    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame1 = env.getTime(frameExpr1_.varName);
        size_t baseFrameRef2 = env.getTime(frameRefExpr2_.varName);
        size_t baseFrameEst2 = env.getTime(frameEstimationExpr2_.varName);
        auto frameOrNull1 = computeRefFrame(frameExpr1_, baseFrame1, stream);
        auto frameOrNullRef2 = computeRefFrame(frameRefExpr2_, baseFrameRef2, stream);
        auto frameOrNullEst2 = computeRefFrame(frameEstimationExpr2_, baseFrameEst2, stream);

        if (!frameOrNull1.has_value() || !frameOrNullRef2.has_value() || !frameOrNullEst2.has_value()) {
            return QualityValue::POS_INF;
        }
        
        size_t frame1 = frameOrNull1.value();
        size_t frameRef2 = frameOrNullRef2.value();
        size_t frameEst2 = frameOrNullEst2.value();
        if (frameEst2 < frameRef2) {
            throw std::invalid_argument("Estimation frame must be after reference frame in predicate: " + toString());
        }

        std::string objId1 = env.getObject(objectVar1_);
        std::string objId2 = env.getObject(objectVar2_);

        const DataObject* obj1 = stream.retrieve(frame1, objId1);
        const DataObject* obj2Ref = stream.retrieve(frameRef2, objId2);

        if (!obj1 || !obj2Ref) {
            return QualityValue::NEG_INF;
        }

        // Estimate position of obj2 at frameEst2 based on its position and velocity at frameRef2
        float deltaTime = stream.getFrame(frameEst2).getTimestamp() - stream.getFrame(frameRef2).getTimestamp();
        glm::vec3 estimatedPos2 = obj2Ref->getPosition() + obj2Ref->getVelocity() * deltaTime;

        // Compute Euclidean distance between obj1 position and estimated obj2 position
        glm::vec3 diff = obj1->getPosition() - estimatedPos2;
        if (abs(diff.z) < 4.0f)
            diff.z = 0.0f;  // Ignore Z-axis for distance computation
        double distance = glm::length(diff);
        return computeRobustness(distance, threshold_, op_);
    }

private:
    FrameExpr frameExpr1_;
    std::string objectVar1_;
    FrameExpr frameRefExpr2_;
    std::string objectVar2_;
    FrameExpr frameEstimationExpr2_;
    ComparisonOp op_;
    double threshold_;
};


// ============================================================================
// Predicates related to Ego vehicle (unified for <, <=, >, >=)
// ============================================================================
/**
 * @brief Predicate for distance to ego vehicle: dist_ego(timeExpr, objectVar) op threshold
 * 
 * Distance is computed between the object's position and the ego vehicle's position.
 */
class DistanceToEgoPredicate : public PredicateFormula {
public:
    DistanceToEgoPredicate(const FrameExpr& frameExpr, const std::string& objectVar,
                          ComparisonOp op, double threshold)
        : frameExpr_(frameExpr), objectVar_(objectVar), op_(op), threshold_(threshold) {}

    std::string toString() const override {
        return "dist_ego(" + frameExpr_.toString() + ", " + objectVar_ + ") " + 
               comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<DistanceToEgoPredicate>(frameExpr_, objectVar_, op_, threshold_);
    }

    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    const std::string& getObjectVar() const { return objectVar_; }
    ComparisonOp getOp() const { return op_; }
    double getThreshold() const { return threshold_; }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        const DataObject* obj = stream.retrieve(frame, objId);
        if (!obj) {
            return QualityValue::NEG_INF;
        }

        EgoObject ego = stream.getFrame(frame).getEgoObject();
        glm::vec3 egoPos = ego.getPosition();
        glm::vec3 objPos = obj->getPosition();
        double distance = glm::length(objPos - egoPos);

        return computeRobustness(distance, threshold_, op_);
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
    ComparisonOp op_;
    double threshold_;
};

/**
 * @brief Predicate for ego view angle to object: angle_ego(timeExpr, objectVar) op threshold
 * 
 * For example: angle_ego(x, id) > -30.0 && angle_ego(x, id) < 30.0
 *   says that the object with id 'id' is within the ego vehicle's forward view of 30 degrees at time 'x'.
 * View angle is computed as the angle between the ego vehicle's forward direction
 * and the vector pointing from the ego vehicle to the object.
 */
class EgoViewAnglePredicate : public PredicateFormula {
public:
    EgoViewAnglePredicate(const FrameExpr& frameExpr, const std::string& objectVar,
                          ComparisonOp op, double threshold)
        : frameExpr_(frameExpr), objectVar_(objectVar), op_(op), threshold_(threshold) {}
    std::string toString() const override {
        return "angle_ego(" + frameExpr_.toString() + ", " + objectVar_ + ") " + 
               comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }
    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }
    FormulaPtr clone() const override {
        return std::make_shared<EgoViewAnglePredicate>(frameExpr_, objectVar_, op_, threshold_);
    }
    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    const std::string& getObjectVar() const { return objectVar_; }
    ComparisonOp getOp() const { return op_; }
    double getThreshold() const { return threshold_; }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        const DataObject* obj = stream.retrieve(frame, objId);
        if (!obj) {
            return QualityValue::NEG_INF;
        }

        EgoObject ego = stream.getFrame(frame).getEgoObject();
        glm::vec3 egoPos = ego.getPosition();
        glm::vec3 objPos = obj->getPosition();
        double angle = glm::degrees(glm::orientedAngle(glm::normalize(objPos - egoPos), glm::normalize(ego.getForwardVector()), glm::vec3(0, 0, 1)));

        return computeRobustness(angle, threshold_, op_);
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
    ComparisonOp op_;
    double threshold_;
};

// ============================================================================
// Speed Predicate (unified for <, <=, >, >=)
// ============================================================================

/**
 * @brief Predicate for speed comparison: speed(timeExpr, objectVar) op threshold
 * 
 * Speed is the magnitude of the velocity vector.
 */
class SpeedPredicate : public PredicateFormula {
public:
    SpeedPredicate(const FrameExpr& frameExpr, const std::string& objectVar,
                   ComparisonOp op, double threshold)
        : frameExpr_(frameExpr), objectVar_(objectVar), op_(op), threshold_(threshold) {}

    std::string toString() const override {
        return "speed(" + frameExpr_.toString() + ", " + objectVar_ + ") " + 
               comparisonOpToString(op_) + " " + std::to_string(threshold_);
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<SpeedPredicate>(frameExpr_, objectVar_, op_, threshold_);
    }

    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    const std::string& getObjectVar() const { return objectVar_; }
    ComparisonOp getOp() const { return op_; }
    double getThreshold() const { return threshold_; }
    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        const DataObject* obj = stream.retrieve(frame, objId);
        if (!obj) {
            return QualityValue::NEG_INF;
        }

        return computeRobustness(obj->getSpeed(), threshold_, op_);
    }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
    ComparisonOp op_;
    double threshold_;
};

/**
 * @brief Predicate to check if an object presents (exists) in the frame
 * 
 * Supports both simple time variables and frame expressions with offsets.
 * Examples: present(x, id), present((x+1), id), present((y-2), obj)
 */
class ObjectPresentPredicate : public PredicateFormula {
public:
    /// Constructor with frame expression for full flexibility
    ObjectPresentPredicate(const FrameExpr& frameExpr, const std::string& objectVar)
        : frameExpr_(frameExpr), objectVar_(objectVar) {}
    
    /// Backward-compatible constructor with simple time variable
    ObjectPresentPredicate(const std::string& timeVar, const std::string& objectVar)
        : frameExpr_(timeVar, 0), objectVar_(objectVar) {}

    std::string toString() const override {
        return "present(" + frameExpr_.toString() + ", " + objectVar_ + ")";
    }

    void accept(FormulaVisitor& visitor) const override {
        visitor.visit(*this);
    }

    FormulaPtr clone() const override {
        return std::make_shared<ObjectPresentPredicate>(frameExpr_, objectVar_);
    }

    QualityValue evaluate(const DataStream& stream, size_t /*frameIndex*/, const Environment& env) const {
        size_t baseFrame = env.getTime(frameExpr_.varName);
        auto frameOrNull = computeRefFrame(frameExpr_, baseFrame, stream);
        if (!frameOrNull.has_value()) {
            return QualityValue::POS_INF;
        }
        size_t frame = frameOrNull.value();

        std::string objId = env.getObject(objectVar_);
        if (stream.hasObject(frame, objId)) {
            return QualityValue::POS_INF;
        }
        return QualityValue::NEG_INF;
    }
    
    /// Get the frame expression
    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    
    /// Get the object variable name
    const std::string& getObjectVar() const { return objectVar_; }

private:
    FrameExpr frameExpr_;
    std::string objectVar_;
};

// ============================================================================
// Predicate Factory Functions
// ============================================================================

namespace predicate {

/// Create class equality predicate: C(timeVar, objectVar) = expectedClass
inline FormulaPtr ClassEquals(const std::string& timeVar, const std::string& objectVar, ObjectClass expectedClass) {
    return std::make_shared<ClassEqualsPredicate>(FrameExpr(timeVar), objectVar, expectedClass);
}
inline FormulaPtr ClassEquals(const FrameExpr& frameExpr, const std::string& objectVar, ObjectClass expectedClass) {
    return std::make_shared<ClassEqualsPredicate>(frameExpr, objectVar, expectedClass);
}
inline FormulaPtr ClassEquals(const std::string& timeVar, const std::string& objectVar, const std::string& classStr) {
    return std::make_shared<ClassEqualsPredicate>(FrameExpr(timeVar), objectVar, stringToObjectClass(classStr));
}

/// Create class inequality predicate: C(timeVar, objectVar) != excludedClass
inline FormulaPtr ClassNotEquals(const std::string& timeVar, const std::string& objectVar, ObjectClass excludedClass) {
    return std::make_shared<ClassNotEqualsPredicate>(timeVar, objectVar, excludedClass);
}
inline FormulaPtr ClassNotEquals(const FrameExpr& frameExpr, const std::string& objectVar, ObjectClass excludedClass) {
    return std::make_shared<ClassNotEqualsPredicate>(frameExpr, objectVar, excludedClass);
}

inline FormulaPtr Probability(const std::string& timeVar, const std::string& objectVar, 
                               ComparisonOp op, double threshold) {
    return std::make_shared<ProbabilityPredicate>(FrameExpr(timeVar), objectVar, op, threshold);
}

/// Create probability greater than predicate: P(timeVar, objectVar) > threshold
inline FormulaPtr ProbabilityGt(const std::string& timeVar, const std::string& objectVar, double threshold) {
    return Probability(timeVar, objectVar, ComparisonOp::GT, threshold);
}

/// Create probability greater or equal predicate: P(timeVar, objectVar) >= threshold
inline FormulaPtr ProbabilityGe(const std::string& timeVar, const std::string& objectVar, double threshold) {
    return Probability(timeVar, objectVar, ComparisonOp::GE, threshold);
}

/// Create probability less than predicate: P(timeVar, objectVar) < threshold
inline FormulaPtr ProbabilityLt(const std::string& timeVar, const std::string& objectVar, double threshold) {
    return Probability(timeVar, objectVar, ComparisonOp::LT, threshold);
}

// --- Distance Predicates ---
/// Distance between same object across frames: dist(t1, t2, id)
inline FormulaPtr Distance(const std::string& timeVar1, const std::string& timeVar2,
                           const std::string& objectVar, ComparisonOp op, double threshold) {
    // Same object at both frames, so objectVar is used for both objectVar1 and objectVar2
    return std::make_shared<DistancePredicate>(FrameExpr(timeVar1), objectVar, FrameExpr(timeVar2), objectVar, op, threshold);
}
inline FormulaPtr Distance(const FrameExpr& frameExpr1, const FrameExpr& frameExpr2,
                           const std::string& objectVar, ComparisonOp op, double threshold) {
    // Same object at both frames, so objectVar is used for both objectVar1 and objectVar2
    return std::make_shared<DistancePredicate>(frameExpr1, objectVar, frameExpr2, objectVar, op, threshold);
}

/// Distance between two different objects: dist_pos(t1, id1, t2, id2)
inline FormulaPtr DistancePos(const std::string& timeVar1, const std::string& objectVar1,
                              const std::string& timeVar2, const std::string& objectVar2,
                              ComparisonOp op, double threshold) {
    return std::make_shared<DistancePredicate>(FrameExpr(timeVar1), objectVar1, FrameExpr(timeVar2), objectVar2, op, threshold);
}
inline FormulaPtr DistancePos(const FrameExpr& frameExpr1, const std::string& objectVar1,
                              const FrameExpr& frameExpr2, const std::string& objectVar2,
                              ComparisonOp op, double threshold) {
    return std::make_shared<DistancePredicate>(frameExpr1, objectVar1, frameExpr2, objectVar2, op, threshold);
}

// Shortcuts for common distance comparisons
inline FormulaPtr DistanceLt(const std::string& t1, const std::string& t2, 
                              const std::string& id, double threshold) {
    return Distance(t1, t2, id, ComparisonOp::LT, threshold);
}
inline FormulaPtr DistanceLt(const std::string& timeVar1, const std::string& timeVar2,
                              const std::string& objectVar1, const std::string& objectVar2,
                              double threshold) {
    return DistancePos(timeVar1, objectVar1, timeVar2, objectVar2, ComparisonOp::LT, threshold);
}

// IOU
inline FormulaPtr IoU(const std::string& timeVar1, const std::string& objectVar1,
                      const std::string& timeVar2, const std::string& objectVar2, 
                      ComparisonOp op, double threshold) {
    // Same object at both frames, so objectVar is used for both objectVar1 and objectVar2
    return std::make_shared<IoUPredicate>(FrameExpr(timeVar1), objectVar1, FrameExpr(timeVar2), objectVar2, op, threshold);
}
inline FormulaPtr IoUGt(const std::string& timeVar1, const std::string& objectVar1,
                         const std::string& timeVar2, const std::string& objectVar2,
                         double threshold) {
    return IoU(timeVar1, objectVar1, timeVar2, objectVar2, ComparisonOp::GT, threshold);
}

/// Create object exists predicate
inline FormulaPtr Exists(const std::string& timeVar, const std::string& objectVar) {
    return std::make_shared<ObjectExistsPredicate>(FrameExpr(timeVar), objectVar);
}

/// Create custom predicate with a scoring function
inline FormulaPtr Custom(const std::string& name, ScoringFunction scoringFn) {
    return std::make_shared<CustomPredicate>(name, std::move(scoringFn));
}

} // namespace predicate

} // namespace tqtl

#endif // TQTL_PREDICATE_HPP
