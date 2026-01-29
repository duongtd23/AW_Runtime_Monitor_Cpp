#ifndef TQTL_MONITOR_HPP
#define TQTL_MONITOR_HPP

#include "Formula.hpp"
#include "Predicate.hpp"
#include "DataStream.hpp"
#include "Environment.hpp"
#include "QualityValue.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace tqtl {

/**
 * @brief Monitor for evaluating TQTL formulas over data streams.
 * 
 * This class implements the quantitative semantics of TQTL as defined in the paper.
 * The monitor computes the quality value (robustness) of a formula at each time step.
 * 
 * Semantics:
 * - [[⊤]](D, i, ε) = +∞
 * - [[¬φ]](D, i, ε) = -[[φ]](D, i, ε)
 * - [[φ₁ ∨ φ₂]](D, i, ε) = max([[φ₁]](D, i, ε), [[φ₂]](D, i, ε))
 * - [[φ₁ U φ₂]](D, i, ε) = max_{i≤j} min([[φ₂]](D, j, ε), min_{i≤k<j} [[φ₁]](D, k, ε))
 * - [[x.φ]](D, i, ε) = [[φ]](D, i, ε[x ⇐ i])
 * - [[∃id@x, φ]](D, i, ε) = max_{k ∈ SO(D(ε(x)))} [[φ]](D, i, ε[id ⇐ k])
 * - [[x ≤ y + n]](D, i, ε) = +∞ if ε(x) ≤ ε(y) + n, else -∞
 */
class Monitor {
public:
    /**
     * @brief Construct a monitor for a given formula.
     * 
     * @param formula The TQTL formula to evaluate
     */
    explicit Monitor(FormulaPtr formula)
        : formula_(std::move(formula)) {}

    /**
     * @brief Evaluate the formula over a data stream starting at frame 0.
     * 
     * @param stream The data stream to evaluate against
     * @return The quality value of the formula
     */
    QualityValue evaluate(const DataStream& stream) const {
        return evaluate(stream, 0, Environment());
    }

    /**
     * @brief Evaluate the formula over a data stream at a specific frame.
     * 
     * @param stream The data stream to evaluate against
     * @param frameIndex The starting frame index
     * @return The quality value of the formula
     */
    QualityValue evaluate(const DataStream& stream, size_t frameIndex) const {
        return evaluate(stream, frameIndex, Environment());
    }

    /**
     * @brief Evaluate the formula over a data stream, but skip k last frames.
     * 
     * @param stream The data stream to evaluate against
     * @param k The number of frames to skip at the end
     * @return The quality value of the formula
     */
    QualityValue evaluateSkippingLastFrames(const DataStream& stream, size_t k, size_t startIndex=0) const {
        if (k >= stream.length()) {
            throw std::invalid_argument("k exceeds stream length");
        }
        DataStream subStream = stream.clone(stream.length() - k, startIndex);
        return evaluate(subStream, 0, Environment());
    }


    /**
     * @brief Evaluate the formula over a data stream with an environment.
     * 
     * This is the main evaluation function that implements the TQTL semantics.
     * Corresponds to [[φ]](D, i, ε) in the paper.
     * 
     * @param stream The data stream (D)
     * @param frameIndex The current frame index (i)
     * @param env The environment (ε)
     * @return The quality value
     */
    QualityValue evaluate(const DataStream& stream, size_t frameIndex, 
                          const Environment& env) const {
        return evaluateFormula(formula_.get(), stream, frameIndex, env);
    }

    /**
     * @brief Evaluate the formula at all frames and return the results.
     * 
     * @param stream The data stream to evaluate against
     * @return Vector of quality values, one for each frame
     */
    std::vector<QualityValue> evaluateAll(const DataStream& stream) const {
        std::vector<QualityValue> results;
        results.reserve(stream.length());
        
        for (size_t i = 0; i < stream.length(); ++i) {
            results.push_back(evaluate(stream, i));
        }
        
        return results;
    }

    /**
     * @brief Check if the formula is satisfied (quality value > 0).
     * 
     * @param stream The data stream to check against
     * @return true if the formula is satisfied, false otherwise
     */
    bool isSatisfied(const DataStream& stream) const {
        QualityValue result = evaluate(stream);
        return result > QualityValue(0.0);
    }

    /**
     * @brief Check if the formula is satisfied with a tolerance.
     * 
     * @param stream The data stream to check against
     * @param tolerance The tolerance for satisfaction
     * @return true if quality value > tolerance
     */
    bool isSatisfied(const DataStream& stream, double tolerance) const {
        QualityValue result = evaluate(stream);
        return result > QualityValue(tolerance);
    }

    /**
     * @brief Get the formula being monitored.
     */
    const FormulaPtr& getFormula() const { return formula_; }

    /**
     * @brief Create a monitor for a formula.
     */
    static Monitor create(FormulaPtr formula) {
        return Monitor(std::move(formula));
    }

private:
    FormulaPtr formula_;

    /**
     * @brief Recursively evaluate a formula.
     * 
     * This dispatches to the appropriate evaluation method based on formula type.
     */
    QualityValue evaluateFormula(const Formula* formula, const DataStream& stream,
                                  size_t frameIndex, const Environment& env) const {
        if (!formula) {
            throw std::invalid_argument("Null formula pointer");
        }

        switch (formula->getType()) {
            case Formula::Type::True:
                return evaluateTrue();
            
            case Formula::Type::Negation:
                return evaluateNegation(
                    static_cast<const NegationFormula*>(formula), stream, frameIndex, env);
            
            case Formula::Type::Disjunction:
                return evaluateDisjunction(
                    static_cast<const DisjunctionFormula*>(formula), stream, frameIndex, env);
            
            case Formula::Type::Until:
                return evaluateUntil(
                    static_cast<const UntilFormula*>(formula), stream, frameIndex, env);
            
            case Formula::Type::Freeze:
                return evaluateFreeze(
                    static_cast<const FreezeFormula*>(formula), stream, frameIndex, env);
            
            case Formula::Type::Exists:
                return evaluateExists(
                    static_cast<const ExistsFormula*>(formula), stream, frameIndex, env);
            
            case Formula::Type::TimeConstraint:
                return evaluateTimeConstraint(
                    static_cast<const TimeConstraintFormula*>(formula), env);
            
            case Formula::Type::Predicate:
                return evaluatePredicate(
                    static_cast<const PredicateFormula*>(formula), stream, frameIndex, env);
            
            default:
                throw std::runtime_error("Unknown formula type");
        }
    }

    /**
     * @brief [[⊤]](D, i, ε) = +∞
     */
    QualityValue evaluateTrue() const {
        return QualityValue::POS_INF;
    }

    /**
     * @brief [[¬φ]](D, i, ε) = -[[φ]](D, i, ε)
     */
    QualityValue evaluateNegation(const NegationFormula* formula, const DataStream& stream,
                                   size_t frameIndex, const Environment& env) const {
        QualityValue operandValue = evaluateFormula(
            formula->getOperand().get(), stream, frameIndex, env);
        return -operandValue;
    }

    /**
     * @brief [[φ₁ ∨ φ₂]](D, i, ε) = max([[φ₁]](D, i, ε), [[φ₂]](D, i, ε))
     */
    QualityValue evaluateDisjunction(const DisjunctionFormula* formula, const DataStream& stream,
                                      size_t frameIndex, const Environment& env) const {
        QualityValue leftValue = evaluateFormula(
            formula->getLeft().get(), stream, frameIndex, env);
        QualityValue rightValue = evaluateFormula(
            formula->getRight().get(), stream, frameIndex, env);
        
        return QualityValue::max(leftValue, rightValue);
    }

    /**
     * @brief [[φ₁ U φ₂]](D, i, ε) = max_{i≤j} min([[φ₂]](D, j, ε), min_{i≤k<j} [[φ₁]](D, k, ε))
     * 
     * φ₁ must hold until φ₂ becomes true.
     * The quality value is the maximum over all possible j of the minimum of:
     *   - φ₂ at time j
     *   - The minimum of φ₁ over all times from i to j-1
     */
    QualityValue evaluateUntil(const UntilFormula* formula, const DataStream& stream,
                                size_t frameIndex, const Environment& env) const {
        if (frameIndex >= stream.length()) {
            return QualityValue::NEG_INF;  // No more frames to satisfy Until
        }

        QualityValue maxValue = QualityValue::NEG_INF;
        QualityValue minPhi1 = QualityValue::POS_INF;  // Running minimum of φ₁

        // For each possible j from i to the end of the stream
        for (size_t j = frameIndex; j < stream.length(); ++j) {
            // Evaluate φ₂ at time j
            QualityValue phi2Value = evaluateFormula(
                formula->getRight().get(), stream, j, env);

            // For j == i (first iteration), minPhi1 doesn't include any time
            // (there's no k such that i ≤ k < j when j == i)
            // So we just use phi2Value directly

            QualityValue combinedValue;
            if (j == frameIndex) {
                // No φ₁ values to consider when j == i
                combinedValue = phi2Value;
            } else {
                // min(φ₂(j), min_{i≤k<j} φ₁(k))
                combinedValue = QualityValue::min(phi2Value, minPhi1);
            }

            // Update the maximum
            maxValue = QualityValue::max(maxValue, combinedValue);

            // Update the running minimum of φ₁ for the next iteration
            // We need to include φ₁(j) in the running min for j+1
            QualityValue phi1Value = evaluateFormula(
                formula->getLeft().get(), stream, j, env);
            minPhi1 = QualityValue::min(minPhi1, phi1Value);
        }

        return maxValue;
    }

    /**
     * @brief [[x.φ]](D, i, ε) = [[φ]](D, i, ε[x ⇐ i])
     * 
     * Freeze the current time in variable x.
     */
    QualityValue evaluateFreeze(const FreezeFormula* formula, const DataStream& stream,
                                 size_t frameIndex, const Environment& env) const {
        // Create new environment with x bound to current frame index
        Environment newEnv = env.withTime(formula->getTimeVar(), frameIndex);
        
        // Evaluate the operand in the new environment
        return evaluateFormula(formula->getOperand().get(), stream, frameIndex, newEnv);
    }

    /**
     * @brief [[∃id@(x+n), φ]](D, i, ε) = max_{k ∈ SO(D(ε(x)+n))} [[φ]](D, i, ε[id <== k])
     * 
     * Existential quantification over objects in the frame referenced by frame expression.
     * The frame expression can include an offset: x, (x+1), (x-2), etc.
     * 
     * Special case: If the computed frame index exceeds the data stream bounds,
     * returns +∞ (true) to handle end-of-stream gracefully. This means
     * "if frame x+n doesn't exist, the property is trivially satisfied."
     */
    QualityValue evaluateExists(const ExistsFormula* formula, const DataStream& stream,
                                 size_t frameIndex, const Environment& env) const {
        // Get the frame index from the frame expression (variable + offset)
        const FrameExpr& frameExpr = formula->getFrameExpr();
        size_t baseFrame = env.getTime(frameExpr.varName);
        
        // Compute the reference frame with offset
        size_t referenceFrame;
        if (frameExpr.offset >= 0) {
            referenceFrame = baseFrame + static_cast<size_t>(frameExpr.offset);
        } else {
            // Handle negative offset - check for underflow
            size_t absOffset = static_cast<size_t>(-frameExpr.offset);
            if (baseFrame < absOffset) {
                // Frame would be negative - return +∞ (trivially satisfied)
                return QualityValue::POS_INF;
            }
            referenceFrame = baseFrame - absOffset;
        }
        
        // Check if reference frame is within stream bounds
        // If out of bounds, return +∞ (property trivially satisfied at end of stream)
        if (referenceFrame >= stream.length()) {
            return QualityValue::POS_INF;
        }
        
        // Get all object IDs in that frame: SO(D(ε(x)+n))
        std::unordered_set<std::string> objectIds = stream.getObjectIds(referenceFrame);
        
        if (objectIds.empty()) {
            return QualityValue::NEG_INF;  // No objects to quantify over
        }

        QualityValue maxValue = QualityValue::NEG_INF;

        // Take the maximum over all objects
        for (std::string objId : objectIds) {
            // Create new environment with object variable bound to this object
            Environment newEnv = env.withObject(formula->getObjectVar(), objId);
            
            // Evaluate the operand in the new environment
            QualityValue value = evaluateFormula(
                formula->getOperand().get(), stream, frameIndex, newEnv);
            
            maxValue = QualityValue::max(maxValue, value);
        }

        return maxValue;
    }

    /**
     * @brief [[x ≤ y + n]](D, i, ε) = +∞ if ε(x) ≤ ε(y) + n, else -∞
     */
    QualityValue evaluateTimeConstraint(const TimeConstraintFormula* formula,
                                         const Environment& env) const {
        size_t leftTime = env.getTime(formula->getLeftVar());
        size_t rightTime = env.getTime(formula->getRightVar());
        int offset = formula->getOffset();

        // Check if ε(x) ≤ ε(y) + n
        // Handle potential underflow when offset is negative
        if (offset >= 0) {
            if (leftTime <= rightTime + static_cast<size_t>(offset)) {
                return QualityValue::POS_INF;
            }
        } else {
            size_t absOffset = static_cast<size_t>(-offset);
            if (rightTime >= absOffset && leftTime <= rightTime - absOffset) {
                return QualityValue::POS_INF;
            } else if (rightTime < absOffset) {
                return QualityValue::NEG_INF;  // rightTime + offset would underflow
            }
        }
        
        return QualityValue::NEG_INF;
    }

    /**
     * @brief Evaluate a predicate formula.
     * 
     * Dispatches to the appropriate predicate's evaluate method.
     */
    QualityValue evaluatePredicate(const PredicateFormula* formula, const DataStream& stream,
                                    size_t frameIndex, const Environment& env) const {
        // Try each concrete predicate type
        
        // CustomPredicate
        if (const auto* custom = dynamic_cast<const CustomPredicate*>(formula)) {
            return custom->evaluate(stream, frameIndex, env);
        }
        
        // ClassEqualsPredicate
        if (const auto* classEquals = dynamic_cast<const ClassEqualsPredicate*>(formula)) {
            return classEquals->evaluate(stream, frameIndex, env);
        }
        
        // ClassNotEqualsPredicate
        if (const auto* classNotEquals = dynamic_cast<const ClassNotEqualsPredicate*>(formula)) {
            return classNotEquals->evaluate(stream, frameIndex, env);
        }
        
        // ProbabilityPredicate
        if (const auto* probPred = dynamic_cast<const ProbabilityPredicate*>(formula)) {
            return probPred->evaluate(stream, frameIndex, env);
        }
        
        // IoUPredicate
        if (const auto* iouPredicate = dynamic_cast<const IoUPredicate*>(formula)) {
            return iouPredicate->evaluate(stream, frameIndex, env);
        }
        
        // ObjectExistsPredicate
        if (const auto* objExists = dynamic_cast<const ObjectExistsPredicate*>(formula)) {
            return objExists->evaluate(stream, frameIndex, env);
        }

        // Distance predicate (unified)
        if (const auto* distPred = dynamic_cast<const DistancePredicate*>(formula)) {
            return distPred->evaluate(stream, frameIndex, env);
        }

        // Speed predicate
        if (const auto* speedPred = dynamic_cast<const SpeedPredicate*>(formula)) {
            return speedPred->evaluate(stream, frameIndex, env);
        }

        // Present predicate
        if (const auto* presentPred = dynamic_cast<const ObjectPresentPredicate*>(formula)) {
            return presentPred->evaluate(stream, frameIndex, env);
        }

        // Ego-related predicates
        if (const auto* egoDistPred = dynamic_cast<const DistanceToEgoPredicate*>(formula)) {
            return egoDistPred->evaluate(stream, frameIndex, env);
        }
        if (const auto* egoAnglePred = dynamic_cast<const EgoViewAnglePredicate*>(formula)) {
            return egoAnglePred->evaluate(stream, frameIndex, env);
        }
        
        throw std::runtime_error("Unknown predicate type: " + formula->toString());
    }
};

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Evaluate a formula over a data stream.
 * 
 * @param formula The formula to evaluate
 * @param stream The data stream
 * @return The quality value
 */
inline QualityValue evaluate(const FormulaPtr& formula, const DataStream& stream) {
    return Monitor(formula).evaluate(stream);
}

/**
 * @brief Evaluate a formula over a data stream at a specific frame.
 * 
 * @param formula The formula to evaluate
 * @param stream The data stream
 * @param frameIndex The starting frame
 * @return The quality value
 */
inline QualityValue evaluate(const FormulaPtr& formula, const DataStream& stream, size_t frameIndex) {
    return Monitor(formula).evaluate(stream, frameIndex);
}

/**
 * @brief Evaluate a formula over a data stream with an environment.
 * 
 * @param formula The formula to evaluate
 * @param stream The data stream
 * @param frameIndex The starting frame
 * @param env The environment
 * @return The quality value
 */
inline QualityValue evaluate(const FormulaPtr& formula, const DataStream& stream,
                              size_t frameIndex, const Environment& env) {
    return Monitor(formula).evaluate(stream, frameIndex, env);
}

/**
 * @brief Evaluate a formula over a data stream, skipping the last k frames.
 * 
 * @param formula The formula to evaluate
 * @param stream The data stream
 * @return The quality value
 */
inline QualityValue evaluateSkippingLastFrames(const FormulaPtr& formula, const DataStream& stream,
                                                size_t k) {
    return Monitor(formula).evaluateSkippingLastFrames(stream, k);
}

/**
 * @brief Check if a formula is satisfied by a data stream.
 * 
 * @param formula The formula to check
 * @param stream The data stream
 * @return true if the formula is satisfied (quality > 0)
 */
inline bool isSatisfied(const FormulaPtr& formula, const DataStream& stream) {
    return Monitor(formula).isSatisfied(stream);
}

}

#endif // TQTL_MONITOR_HPP
