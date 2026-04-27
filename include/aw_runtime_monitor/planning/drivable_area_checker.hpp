#ifndef EXTERNAL_DRIVABLE_AREA_CHECKER_HPP
#define EXTERNAL_DRIVABLE_AREA_CHECKER_HPP

#include "autoware/route_handler/route_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "nav_msgs/msg/odometry.hpp"

class ExternalDrivableAreaChecker
{
public:
    ExternalDrivableAreaChecker() {}

    /**
     * @brief Get expanded drivable area including:
     *        1. Current lanelet and its left/right adjacent lanes
     *        2. Previous lanelets (within backward_distance)
     *        3. Next lanelets (within forward_distance)
     *        4. Adjacent lanes of previous/next lanelets
     */
    struct DrivableAreaBounds {
        std::vector<geometry_msgs::msg::Point> left_bound;
        std::vector<geometry_msgs::msg::Point> right_bound;
    };

    DrivableAreaBounds getExpandedDrivableArea(
        geometry_msgs::msg::Pose current_pose,
        const double backward_distance = 5.0,
        const double forward_distance = 80.0) {
        DrivableAreaBounds result;

        if (!route_handler_.isHandlerReady()) {
            return result;
        }
        lanelet::ConstLanelet current_lanelet;
        if (!route_handler_.getClosestLaneletWithinRoute(current_pose, &current_lanelet)) {
            std::cout << "[WARNING] Failed to find closest lanelet" << std::endl;
            return result;
        }

    // Get lanelet sequence (current + previous + next lanelets along the route)
    lanelet::ConstLanelets lanelet_sequence = route_handler_.getLaneletSequence(
      current_lanelet, current_pose, backward_distance, forward_distance);

    if (lanelet_sequence.empty()) {
      std::cout << "[WARNING] Lanelet sequence is empty" << std::endl;
      return result;
    }

    // For each lanelet in sequence, get expanded lanes (including adjacent)
    std::vector<lanelet::ConstLanelet> all_leftmost_lanelets;
    std::vector<lanelet::ConstLanelet> all_rightmost_lanelets;

    for (const auto & lanelet : lanelet_sequence) {
        // Get all adjacent lanes including current lane
        // getAllSharedLineStringLanelets returns lanelets sharing linestrings
        // Parameters: (lanelet, search_right, search_left, include_opposite, invert_opposite)
        // lanelet::ConstLanelets all_adjacent = route_handler_.getAllSharedLineStringLanelets(
        //     lanelet, 
        //     true,   // search right
        //     true,   // search left
        //     false,  // don't include opposite direction
        //     false   // don't invert
        // );

        // Get the leftmost and rightmost
        lanelet::ConstLanelet leftmost = route_handler_.getMostLeftLanelet(
            lanelet, false, true);  // enable_same_root=false, get_shoulder_lane=true
        lanelet::ConstLanelet rightmost = route_handler_.getMostRightLanelet(
            lanelet, false, true);

        all_leftmost_lanelets.push_back(leftmost);
        all_rightmost_lanelets.push_back(rightmost);
        }

        // Extract bounds from leftmost lanelets (left bound)
        for (const auto & lanelet : all_leftmost_lanelets) {
            for (const auto & pt : lanelet.leftBound()) {
                geometry_msgs::msg::Point p;
                p.x = pt.x();
                p.y = pt.y();
                p.z = pt.z();
                result.left_bound.push_back(p);
            }
        }

        // Extract bounds from rightmost lanelets (right bound)
        for (const auto & lanelet : all_rightmost_lanelets) {
            for (const auto & pt : lanelet.rightBound()) {
                geometry_msgs::msg::Point p;
                p.x = pt.x();
                p.y = pt.y();
                p.z = pt.z();
                result.right_bound.push_back(p);
            }
        }

        // Remove duplicate points (lanelets share boundary points)
        removeDuplicatePoints(result.left_bound);
        removeDuplicatePoints(result.right_bound);

        return result;
    }

    /**
     * @brief Check if a point is within the drivable area polygon
     */
    bool isPointInDrivableArea(
        const geometry_msgs::msg::Point & point,
        const DrivableAreaBounds & bounds) const {
        if (bounds.left_bound.empty() || bounds.right_bound.empty()) {
            std::cout << "[WARNING] Drivable area bounds are empty" << std::endl;
            return false;
        }

        std::vector<geometry_msgs::msg::Point> polygon;
        for (const auto & p : bounds.left_bound) {
            polygon.push_back(p);
        }
        for (auto it = bounds.right_bound.rbegin(); it != bounds.right_bound.rend(); ++it) {
            polygon.push_back(*it);
        }
        
        // Close polygon
        if (!polygon.empty()) {
            polygon.push_back(polygon.front());
        }
        return isPointInPolygon(point, polygon);
    }
    
    /**
     * @brief Check if a point (x, y) is within the cached drivable area
     * Note: Must call cacheCurrentDrivableArea() first
     */
    bool isPointInDrivableArea(double x, double y) const {
        if (cached_bounds_.left_bound.empty() || cached_bounds_.right_bound.empty()) {
            std::cout << "[WARNING] Drivable area bounds are empty." << std::endl;
            return true;  // If no bounds cached, assume point is valid
        }
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        return isPointInDrivableArea(point, cached_bounds_);
    }
    
    /**
     * @brief Cache drivable area bounds for a given pose
     */
    void cacheCurrentDrivableArea(
        const geometry_msgs::msg::Pose& current_pose,
        double backward_distance = 5.0,
        double forward_distance = 80.0) {
        cached_bounds_ = getExpandedDrivableArea(current_pose, backward_distance, forward_distance);
        // std::cout << "[INFO] Cached drivable area bounds for current pose (" << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << ").\nLeft bound: ";
        // for (const auto & p : cached_bounds_.left_bound) {
        //     std::cout << "(" << p.x << ", " << p.y << ", " << p.z << "),";
        // }
        // std::cout << "\nRight bound: ";
        // for (const auto & p : cached_bounds_.right_bound) {
        //     std::cout << "(" << p.x << ", " << p.y << ", " << p.z << "),";
        // }
        // std::cout << std::endl;
    }
    
    /**
     * @brief Get the cached drivable area bounds
     */
    const DrivableAreaBounds& getCachedBounds() const {
        return cached_bounds_;
    }

    void setMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg) {
        route_handler_.setMap(*map_msg);
    }
    void setRoute(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_msg) {
        route_handler_.setRoute(*route_msg);
    }
    void clearRoute() {
        route_handler_.clearRoute();
    }
private:
    autoware::route_handler::RouteHandler route_handler_;
    DrivableAreaBounds cached_bounds_;

    void removeDuplicatePoints(std::vector<geometry_msgs::msg::Point> & points){
        if (points.size() < 2) return;
        
        constexpr double eps = 0.2;
        auto new_end = std::unique(points.begin(), points.end(),
        [eps](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
            return std::abs(a.x - b.x) < eps && 
                std::abs(a.y - b.y) < eps && 
                std::abs(a.z - b.z) < eps;
        });
        points.erase(new_end, points.end());
    }

    bool isPointInPolygon(
        const geometry_msgs::msg::Point & point,
        const std::vector<geometry_msgs::msg::Point> & polygon) const {
        // Ray casting algorithm for point-in-polygon test
        if (polygon.size() < 3) return false;

        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const auto & pi = polygon[i];
            const auto & pj = polygon[j];
            
            if (((pi.y > point.y) != (pj.y > point.y)) &&
                (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)) {
                inside = !inside;
            }
        }
        return inside;
    }
};

#endif  // EXTERNAL_DRIVABLE_AREA_CHECKER_HPP