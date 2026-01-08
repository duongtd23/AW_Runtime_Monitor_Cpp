#ifndef TQTL_FORMULA_HPP
#define TQTL_FORMULA_HPP

#include "QualityValue.hpp"
#include "DataStream.hpp"
#include "Environment.hpp"
#include <memory>
#include <string>
#include <vector>
#include <ostream>
#include <functional>

namespace tqtl {

class Formula;
class FormulaVisitor;

/// Shared pointer type for formulas
using FormulaPtr = std::shared_ptr<Formula>;

/**
 * @brief Frame expression representing a time variable with optional offset.
 * 
 * The variable+offset notation is an extension to the TQTL grammar to allow for more flexible frame lookups.
 * Used in quantifiers to specify which frame to look up objects from.
 * Examples: x, (x+1), (x-1), (y+5)
 */
struct FrameExpr {
    std::string varName;  ///< Time variable name
    int offset;           ///< Offset from the variable (can be positive, negative, or 0). Again, this is an extension to the TQTL grammar.
    
    FrameExpr(const std::string& var = "", int off = 0) : varName(var), offset(off) {}
    
    std::string toString() const {
        if (offset == 0) return varName;
        if (offset > 0) return "(" + varName + "+" + std::to_string(offset) + ")";
        return "(" + varName + std::to_string(offset) + ")";  // offset is negative, includes minus sign
    }
    
    bool operator==(const FrameExpr& other) const {
        return varName == other.varName && offset == other.offset;
    }
};

/**
 * @brief Abstract base class for all TQTL formula nodes.
 * 
 * This class represents the abstract syntax tree (AST) for TQTL formulas.
 * Each concrete formula type inherits from this class.
 * 
 * TQTL Grammar (from the paper):
 *   φ ::= ⊤ | π | x.φ | ∃id@x, φ | x ≤ y + n | ¬φ | φ₁ ∨ φ₂ | φ₁ U φ₂
 */
class Formula : public std::enable_shared_from_this<Formula> {
public:
    /// Formula type enumeration
    enum class Type {
        True,           // ⊤
        Predicate,      // π
        Freeze,         // x.φ
        Exists,         // ∃id@x, φ
        TimeConstraint, // x ≤ y + n
        Negation,       // ¬φ
        Disjunction,    // φ₁ ∨ φ₂
        Until           // φ₁ U φ₂
    };

    virtual ~Formula() = default;

    virtual Type getType() const = 0;

    /// Get string representation of the formula type
    virtual std::string getTypeName() const = 0;

    /// Convert formula to string representation
    virtual std::string toString() const = 0;

    /// Accept a visitor (for visitor pattern)
    virtual void accept(FormulaVisitor& visitor) const = 0;

    /// Clone the formula (deep copy)
    virtual FormulaPtr clone() const = 0;

    /// Stream output operator
    friend std::ostream& operator<<(std::ostream& os, const Formula& formula) {
        os << formula.toString();
        return os;
    }
};

/**
 * @brief True formula (⊤)
 * 
 * Semantics: [[⊤]](D, i, ε) = +∞
 */
class TrueFormula : public Formula {
public:
    TrueFormula() = default;

    Type getType() const override { return Type::True; }
    std::string getTypeName() const override { return "True"; }
    std::string toString() const override { return "T"; }
    void accept(FormulaVisitor& visitor) const override;
    FormulaPtr clone() const override { return std::make_shared<TrueFormula>(); }
};

/**
 * @brief Negation formula (¬φ)
 * 
 * Semantics: [[¬φ]](D, i, ε) = -[[φ]](D, i, ε)
 */
class NegationFormula : public Formula {
public:
    explicit NegationFormula(FormulaPtr operand)
        : operand_(std::move(operand)) {}

    Type getType() const override { return Type::Negation; }
    std::string getTypeName() const override { return "Negation"; }
    
    std::string toString() const override {
        return "¬(" + operand_->toString() + ")";
    }

    void accept(FormulaVisitor& visitor) const override;
    
    FormulaPtr clone() const override {
        return std::make_shared<NegationFormula>(operand_->clone());
    }

    /// Get the operand formula
    const FormulaPtr& getOperand() const { return operand_; }

private:
    FormulaPtr operand_;
};

/**
 * @brief Disjunction formula (φ₁ ∨ φ₂)
 * 
 * Semantics: [[φ₁ ∨ φ₂]](D, i, ε) = max([[φ₁]](D, i, ε), [[φ₂]](D, i, ε))
 */
class DisjunctionFormula : public Formula {
public:
    DisjunctionFormula(FormulaPtr left, FormulaPtr right)
        : left_(std::move(left)), right_(std::move(right)) {}

    Type getType() const override { return Type::Disjunction; }
    std::string getTypeName() const override { return "Disjunction"; }
    
    std::string toString() const override {
        return "(" + left_->toString() + " ∨ " + right_->toString() + ")";
    }

    void accept(FormulaVisitor& visitor) const override;
    
    FormulaPtr clone() const override {
        return std::make_shared<DisjunctionFormula>(left_->clone(), right_->clone());
    }

    /// Get the left operand
    const FormulaPtr& getLeft() const { return left_; }
    
    /// Get the right operand
    const FormulaPtr& getRight() const { return right_; }

private:
    FormulaPtr left_;
    FormulaPtr right_;
};

/**
 * @brief Until formula (φ₁ U φ₂)
 * 
 * Semantics: [[φ₁ U φ₂]](D, i, ε) = max_{i≤j} min([[φ₂]](D, j, ε), min_{i≤k<j} [[φ₁]](D, k, ε))
 */
class UntilFormula : public Formula {
public:
    UntilFormula(FormulaPtr left, FormulaPtr right)
        : left_(std::move(left)), right_(std::move(right)) {}

    Type getType() const override { return Type::Until; }
    std::string getTypeName() const override { return "Until"; }
    
    std::string toString() const override {
        return "(" + left_->toString() + " U " + right_->toString() + ")";
    }

    void accept(FormulaVisitor& visitor) const override;
    
    FormulaPtr clone() const override {
        return std::make_shared<UntilFormula>(left_->clone(), right_->clone());
    }

    /// Get the left operand (φ₁)
    const FormulaPtr& getLeft() const { return left_; }
    
    /// Get the right operand (φ₂)
    const FormulaPtr& getRight() const { return right_; }

private:
    FormulaPtr left_;
    FormulaPtr right_;
};

/**
 * @brief Freeze time quantifier (x.φ)
 * 
 * Semantics: [[x.φ]](D, i, ε) = [[φ]](D, i, ε[x <== i])
 * 
 * Assigns the current frame index i to time variable x before evaluating φ.
 */
class FreezeFormula : public Formula {
public:
    FreezeFormula(const std::string& timeVar, FormulaPtr operand)
        : timeVar_(timeVar), operand_(std::move(operand)) {}

    Type getType() const override { return Type::Freeze; }
    std::string getTypeName() const override { return "Freeze"; }
    
    std::string toString() const override {
        return timeVar_ + ".(" + operand_->toString() + ")";
    }

    void accept(FormulaVisitor& visitor) const override;
    
    FormulaPtr clone() const override {
        return std::make_shared<FreezeFormula>(timeVar_, operand_->clone());
    }

    /// Get the time variable name
    const std::string& getTimeVar() const { return timeVar_; }
    
    /// Get the operand formula
    const FormulaPtr& getOperand() const { return operand_; }

private:
    std::string timeVar_;
    FormulaPtr operand_;
};

/**
 * @brief Existential quantifier over objects (∃id@x, φ) or (∃id@(x+n), φ)
 * 
 * Semantics: [[∃id@(x+n), φ]](D, i, ε) = max_{k ∈ SO(D(ε(x)+n))} [[φ]](D, i, ε[id <== k])
 * 
 * Quantifies over all object IDs in the frame specified by the frame expression.
 * The frame expression can be a simple variable (x) or variable with offset (x+1, x-2).
 * 
 * Special case: If the computed frame index exceeds the data stream size,
 * the quantifier returns +∞ (satisfied) to handle end-of-stream gracefully.
 */
class ExistsFormula : public Formula {
public:
    /// Constructor with FrameExpr for full flexibility
    ExistsFormula(const std::string& objectVar, const FrameExpr& frameExpr, FormulaPtr operand)
        : objectVar_(objectVar), frameExpr_(frameExpr), operand_(std::move(operand)) {}

    /// Backward-compatible constructor with simple time variable (offset = 0)
    ExistsFormula(const std::string& objectVar, const std::string& timeVar, FormulaPtr operand)
        : objectVar_(objectVar), frameExpr_(timeVar, 0), operand_(std::move(operand)) {}

    Type getType() const override { return Type::Exists; }
    std::string getTypeName() const override { return "Exists"; }
    
    std::string toString() const override {
        return "∃" + objectVar_ + "@" + frameExpr_.toString() + ".(" + operand_->toString() + ")";
    }

    void accept(FormulaVisitor& visitor) const override;
    
    FormulaPtr clone() const override {
        return std::make_shared<ExistsFormula>(objectVar_, frameExpr_, operand_->clone());
    }

    /// Get the object variable name
    const std::string& getObjectVar() const { return objectVar_; }
    
    /// Get the time variable name (for backward compatibility)
    const std::string& getTimeVar() const { return frameExpr_.varName; }
    
    /// Get the full frame expression (variable + offset)
    const FrameExpr& getFrameExpr() const { return frameExpr_; }
    
    /// Get the operand formula
    const FormulaPtr& getOperand() const { return operand_; }

private:
    std::string objectVar_;
    FrameExpr frameExpr_;
    FormulaPtr operand_;
};

/**
 * @brief Time constraint formula (x ≤ y + n)
 * 
 * Semantics: [[x ≤ y + n]](D, i, ε) = +∞ if ε(x) ≤ ε(y) + n, else -∞
 */
class TimeConstraintFormula : public Formula {
public:
    TimeConstraintFormula(const std::string& leftVar, const std::string& rightVar, int offset)
        : leftVar_(leftVar), rightVar_(rightVar), offset_(offset) {}

    Type getType() const override { return Type::TimeConstraint; }
    std::string getTypeName() const override { return "TimeConstraint"; }
    
    std::string toString() const override {
        std::string result = leftVar_ + " ≤ " + rightVar_;
        if (offset_ > 0) {
            result += " + " + std::to_string(offset_);
        } else if (offset_ < 0) {
            result += " - " + std::to_string(-offset_);
        }
        return result;
    }

    void accept(FormulaVisitor& visitor) const override;
    
    FormulaPtr clone() const override {
        return std::make_shared<TimeConstraintFormula>(leftVar_, rightVar_, offset_);
    }

    /// Get the left time variable
    const std::string& getLeftVar() const { return leftVar_; }
    
    /// Get the right time variable
    const std::string& getRightVar() const { return rightVar_; }
    
    /// Get the offset (n)
    int getOffset() const { return offset_; }

private:
    std::string leftVar_;
    std::string rightVar_;
    int offset_;
};

/**
 * @brief Abstract base class for predicates (π)
 * 
 * Predicates evaluate object data values and return quality values.
 * Each predicate has an associated scoring function.
 * This is an abstract class - concrete predicates will be defined in Predicate.hpp
 */
class PredicateFormula : public Formula {
public:
    Type getType() const override { return Type::Predicate; }
    std::string getTypeName() const override { return "Predicate"; }
    
    // These must be implemented by concrete predicate classes
    std::string toString() const override = 0;
    void accept(FormulaVisitor& visitor) const override = 0;
    FormulaPtr clone() const override = 0;
};

/**
 * @brief Visitor interface for formula traversal
 */
class FormulaVisitor {
public:
    virtual ~FormulaVisitor() = default;
    
    virtual void visit(const TrueFormula& formula) = 0;
    virtual void visit(const NegationFormula& formula) = 0;
    virtual void visit(const DisjunctionFormula& formula) = 0;
    virtual void visit(const UntilFormula& formula) = 0;
    virtual void visit(const FreezeFormula& formula) = 0;
    virtual void visit(const ExistsFormula& formula) = 0;
    virtual void visit(const TimeConstraintFormula& formula) = 0;
    virtual void visit(const PredicateFormula& formula) = 0;
};

// Implement accept methods
inline void TrueFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }
inline void NegationFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }
inline void DisjunctionFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }
inline void UntilFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }
inline void FreezeFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }
inline void ExistsFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }
inline void TimeConstraintFormula::accept(FormulaVisitor& visitor) const { visitor.visit(*this); }

// ============================================================================
// Formula Factory Functions (for convenient formula construction)
// ============================================================================

namespace formula {

inline FormulaPtr True() {
    return std::make_shared<TrueFormula>();
}
inline FormulaPtr False() {
    return std::make_shared<NegationFormula>(True());
}
inline FormulaPtr Not(FormulaPtr operand) {
    return std::make_shared<NegationFormula>(std::move(operand));
}

inline FormulaPtr Or(FormulaPtr left, FormulaPtr right) {
    return std::make_shared<DisjunctionFormula>(std::move(left), std::move(right));
}
inline FormulaPtr And(FormulaPtr left, FormulaPtr right) {
    return Not(Or(Not(std::move(left)), Not(std::move(right))));
}

/// Create Implication formula (φ₁ --> φ₂ ≡ ¬φ₁ ∨ φ₂)
inline FormulaPtr Implies(FormulaPtr left, FormulaPtr right) {
    return Or(Not(std::move(left)), std::move(right));
}

/// Create Until formula (φ₁ U φ₂)
inline FormulaPtr Until(FormulaPtr left, FormulaPtr right) {
    return std::make_shared<UntilFormula>(std::move(left), std::move(right));
}

/// Create Eventually formula (◇φ ≡ ⊤ U φ)
inline FormulaPtr Eventually(FormulaPtr operand) {
    return Until(True(), std::move(operand));
}

/// Create Always formula (□φ ≡ ¬◇¬φ)
inline FormulaPtr Always(FormulaPtr operand) {
    return Not(Eventually(Not(std::move(operand))));
}

/// Create Freeze time quantifier (x.φ)
inline FormulaPtr Freeze(const std::string& timeVar, FormulaPtr operand) {
    return std::make_shared<FreezeFormula>(timeVar, std::move(operand));
}

/// Create Existential quantifier (∃id@x, φ) - simple time variable version
inline FormulaPtr Exists(const std::string& objectVar, const std::string& timeVar, FormulaPtr operand) {
    return std::make_shared<ExistsFormula>(objectVar, timeVar, std::move(operand));
}

/// Create Existential quantifier (∃id@(x+n), φ) - with frame expression
inline FormulaPtr Exists(const std::string& objectVar, const FrameExpr& frameExpr, FormulaPtr operand) {
    return std::make_shared<ExistsFormula>(objectVar, frameExpr, std::move(operand));
}

/// Create Universal quantifier (∀id@x, φ ≡ ¬(∃id@x, ¬φ)) - simple time variable version
inline FormulaPtr ForAll(const std::string& objectVar, const std::string& timeVar, FormulaPtr operand) {
    return Not(Exists(objectVar, timeVar, Not(std::move(operand))));
}

/// Create Universal quantifier (∀id@(x+n), φ ≡ ¬(∃id@(x+n), ¬φ)) - with frame expression
inline FormulaPtr ForAll(const std::string& objectVar, const FrameExpr& frameExpr, FormulaPtr operand) {
    return Not(Exists(objectVar, frameExpr, Not(std::move(operand))));
}

/// Create Time constraint (x ≤ y + n)
inline FormulaPtr TimeLeq(const std::string& leftVar, const std::string& rightVar, int offset = 0) {
    return std::make_shared<TimeConstraintFormula>(leftVar, rightVar, offset);
}

/// Create Time constraint (x < y + n ≡ x ≤ y + (n-1)) -- only for integer time
inline FormulaPtr TimeLt(const std::string& leftVar, const std::string& rightVar, int offset = 0) {
    return TimeLeq(leftVar, rightVar, offset - 1);
}

/// Create Time constraint (x ≥ y + n ≡ ¬(x ≤ y + (n-1)))
inline FormulaPtr TimeGeq(const std::string& leftVar, const std::string& rightVar, int offset = 0) {
    return Not(TimeLeq(leftVar, rightVar, offset - 1));
}

/// Create Time constraint (x > y + n ≡ ¬(x ≤ y + n))
inline FormulaPtr TimeGt(const std::string& leftVar, const std::string& rightVar, int offset = 0) {
    return Not(TimeLeq(leftVar, rightVar, offset));
}

/// Create Time constraint (x == y + n ≡ x ≤ y + n ∧ x ≥ y + n)
inline FormulaPtr TimeEq(const std::string& leftVar, const std::string& rightVar, int offset = 0) {
    return And(TimeLeq(leftVar, rightVar, offset), TimeGeq(leftVar, rightVar, offset));
}

} // namespace formula

} // namespace tqtl

#endif // TQTL_FORMULA_HPP
