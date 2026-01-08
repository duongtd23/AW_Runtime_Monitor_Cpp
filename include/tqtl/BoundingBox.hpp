#ifndef TQTL_BOUNDING_BOX_HPP
#define TQTL_BOUNDING_BOX_HPP

#include <cmath>
#include <ostream>
#include <string>

namespace tqtl {

/**
 * @brief Represents a bounding box for detected objects.
 * 
 * The bounding box is defined by four coordinates: top, left, bottom, right.
 * These represent pixel coordinates in an image.
 */
class BoundingBox {
public:
    BoundingBox() : top_(0), left_(0), bottom_(0), right_(0) {}

    BoundingBox(int top, int left, int bottom, int right)
        : top_(top), left_(left), bottom_(bottom), right_(right) {}

    int getTop() const { return top_; }
    int getLeft() const { return left_; }
    int getBottom() const { return bottom_; }
    int getRight() const { return right_; }

    int getWidth() const { return right_ - left_; }
    int getHeight() const { return bottom_ - top_; }
    int getArea() const { return getWidth() * getHeight(); }

    double getCenterX() const { return (left_ + right_) / 2.0; }
    double getCenterY() const { return (top_ + bottom_) / 2.0; }

    double centerDistance(const BoundingBox& other) const {
        double dx = getCenterX() - other.getCenterX();
        double dy = getCenterY() - other.getCenterY();
        return std::sqrt(dx * dx + dy * dy);
    }

    /// Calculate Intersection over Union (IoU) with another bounding box
    double iou(const BoundingBox& other) const {
        int intersectTop = std::max(top_, other.top_);
        int intersectLeft = std::max(left_, other.left_);
        int intersectBottom = std::min(bottom_, other.bottom_);
        int intersectRight = std::min(right_, other.right_);

        if (intersectTop >= intersectBottom || intersectLeft >= intersectRight) {
            return 0.0; // No intersection
        }

        int intersectArea = (intersectBottom - intersectTop) * (intersectRight - intersectLeft);
        int unionArea = getArea() + other.getArea() - intersectArea;

        if (unionArea <= 0) return 0.0;
        return static_cast<double>(intersectArea) / unionArea;
    }

    bool operator==(const BoundingBox& other) const {
        return top_ == other.top_ && left_ == other.left_ &&
               bottom_ == other.bottom_ && right_ == other.right_;
    }

    bool operator!=(const BoundingBox& other) const {
        return !(*this == other);
    }

    std::string toString() const {
        return "[" + std::to_string(top_) + ", " + std::to_string(left_) + ", " +
               std::to_string(bottom_) + ", " + std::to_string(right_) + "]";
    }

    /// Stream output operator
    friend std::ostream& operator<<(std::ostream& os, const BoundingBox& bbox) {
        os << bbox.toString();
        return os;
    }

private:
    int top_;
    int left_;
    int bottom_;
    int right_;
};

}
#endif // TQTL_BOUNDING_BOX_HPP
