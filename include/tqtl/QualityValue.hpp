#ifndef TQTL_QUALITY_VALUE_HPP
#define TQTL_QUALITY_VALUE_HPP

#include <limits>
#include <cmath>
#include <ostream>
#include <string>

namespace tqtl {

/**
 * @brief Represents a quality value in TQTL semantics.
 * 
 * Quality values are extended real numbers that include +∞ and -∞.
 * A formula is satisfied iff its quality value > 0.
 */
class QualityValue {
public:
    /// Positive infinity constant
    static const QualityValue POS_INF;
    /// Negative infinity constant
    static const QualityValue NEG_INF;
    static const QualityValue ZERO;

    QualityValue() : value_(0.0), isPositiveInf_(false), isNegativeInf_(false) {}

    explicit QualityValue(double value) 
        : value_(value), isPositiveInf_(false), isNegativeInf_(false) {
        if (std::isinf(value) && value > 0) {
            isPositiveInf_ = true;
            value_ = 0.0;
        } else if (std::isinf(value) && value < 0) {
            isNegativeInf_ = true;
            value_ = 0.0;
        }
    }

    bool isPositiveInfinity() const { return isPositiveInf_; }
    bool isNegativeInfinity() const { return isNegativeInf_; }
    bool isInfinity() const { return isPositiveInf_ || isNegativeInf_; }
    bool isFinite() const { return !isInfinity(); }

    double getValue() const { return value_; }

    /// Check if the formula is satisfied (quality > 0) - for strict predicates (>)
    bool isSatisfied() const {
        if (isPositiveInf_) return true;
        if (isNegativeInf_) return false;
        return value_ > 0.0;
    }

    /// Check if the formula is weakly satisfied (quality >= 0) - for non-strict predicates (>=)
    bool isWeaklySatisfied() const {
        if (isPositiveInf_) return true;
        if (isNegativeInf_) return false;
        return value_ >= 0.0;
    }

    // Arithmetic operations

    /// Negation (for ¬φ semantics)
    QualityValue operator-() const {
        if (isPositiveInf_) return NEG_INF;
        if (isNegativeInf_) return POS_INF;
        return QualityValue(-value_);
    }

    /// Addition
    QualityValue operator+(const QualityValue& other) const {
        if (isPositiveInf_ && other.isNegativeInf_) return ZERO; // undefined, return 0
        if (isNegativeInf_ && other.isPositiveInf_) return ZERO; // undefined, return 0
        if (isPositiveInf_ || other.isPositiveInf_) return POS_INF;
        if (isNegativeInf_ || other.isNegativeInf_) return NEG_INF;
        return QualityValue(value_ + other.value_);
    }

    /// Subtraction
    QualityValue operator-(const QualityValue& other) const {
        return *this + (-other);
    }

    // Comparison operations
    bool operator<(const QualityValue& other) const {
        if (isNegativeInf_ && !other.isNegativeInf_) return true;
        if (isPositiveInf_ || other.isNegativeInf_) return false;
        if (other.isPositiveInf_) return true;
        return value_ < other.value_;
    }

    bool operator>(const QualityValue& other) const {
        return other < *this;
    }

    bool operator<=(const QualityValue& other) const {
        return !(other < *this);
    }

    bool operator>=(const QualityValue& other) const {
        return !(*this < other);
    }

    bool operator==(const QualityValue& other) const {
        if (isPositiveInf_ && other.isPositiveInf_) return true;
        if (isNegativeInf_ && other.isNegativeInf_) return true;
        if (isInfinity() || other.isInfinity()) return false;
        return std::abs(value_ - other.value_) < 1e-10;
    }

    bool operator!=(const QualityValue& other) const {
        return !(*this == other);
    }

    // Static utility functions
    static QualityValue max(const QualityValue& a, const QualityValue& b) {
        return (a > b) ? a : b;
    }
    static QualityValue min(const QualityValue& a, const QualityValue& b) {
        return (a < b) ? a : b;
    }

    std::string toString() const {
        if (isPositiveInf_) return "+∞";
        if (isNegativeInf_) return "-∞";
        return std::to_string(value_);
    }
    friend std::ostream& operator<<(std::ostream& os, const QualityValue& qv) {
        os << qv.toString();
        return os;
    }

private:
    double value_;
    bool isPositiveInf_;
    bool isNegativeInf_;

    /// Private constructor for creating infinity values
    QualityValue(double value, bool posInf, bool negInf)
        : value_(value), isPositiveInf_(posInf), isNegativeInf_(negInf) {}
};

// Define static constants
inline const QualityValue QualityValue::POS_INF = QualityValue(0.0, true, false);
inline const QualityValue QualityValue::NEG_INF = QualityValue(0.0, false, true);
inline const QualityValue QualityValue::ZERO = QualityValue(0.0);

}

#endif // TQTL_QUALITY_VALUE_HPP
