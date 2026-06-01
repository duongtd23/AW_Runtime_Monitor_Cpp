#ifndef UTILS_HPP
#define UTILS_HPP

#include <glm/glm.hpp>  // for vector computation
#include <glm/gtx/norm.hpp> 
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <vector>
#include <cmath>

// Return the center point of the vehicle in world coordinates.
// heading_angle_degree in counter-clockwise degree
inline glm::vec2 getCenterRect(const glm::vec2& position, double heading_angle_degree, const glm::vec2& center_offset) {
    // need to change sign because of different convention
    double theta = -glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
             sin(theta),  cos(theta));
    return position + rot * center_offset;
}


inline std::vector<glm::vec2> getEgoWorldVertices(const glm::vec2& position,
                                         double heading_angle_degree,
                                         const glm::vec2& size,          // (length, width)
                                         const glm::vec2& center_offset) // (x, y)
{
    double length = size.x;
    double width = size.y;
    double dx = length / 2.0;
    double dy = width / 2.0;

    // Local rectangle corners (FR, FL, RL, RR)
    std::vector<glm::vec2> local_vertices = {
        { dx,  dy},
        { dx, -dy},
        {-dx, -dy},
        {-dx,  dy} 
    };

    // Rotation matrix (counter-clockwise)
    // remember to change sign due to different convention
    double theta = -glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
             sin(theta),  cos(theta));

    // Center in world coordinates
    glm::vec2 center_world = getCenterRect(position, heading_angle_degree, center_offset);

    // Rotate and translate each vertex
    std::vector<glm::vec2> world_vertices;
    world_vertices.reserve(local_vertices.size());

    for (const auto& v_local : local_vertices) {
        glm::vec2 v_world = rot * v_local + center_world;
        world_vertices.push_back(v_world);
    }
    return world_vertices;
}

inline std::vector<glm::vec2> getEgoLeftVertices(const glm::vec2& position,
                                     double heading_angle_degree,
                                     const glm::vec2& size,          // (length, width)
                                     const glm::vec2& center_offset) // (x, y)
{
    double length = size.x;
    double width = size.y;
    double dx = length / 2.0;
    double dy = width / 2.0;

    // Local rectangle corners (FL, RL)
    std::vector<glm::vec2> local_vertices = {
        { dx, -dy},
        {-dx, -dy} 
    };

    // Rotation matrix (counter-clockwise)
    // remember to change sign due to different convention
    double theta = -glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
             sin(theta),  cos(theta));

    // Center in world coordinates
    glm::vec2 center_world = getCenterRect(position, heading_angle_degree, center_offset);

    // Rotate and translate each vertex
    std::vector<glm::vec2> world_vertices;
    world_vertices.reserve(local_vertices.size());

    for (const auto& v_local : local_vertices) {
        glm::vec2 v_world = rot * v_local + center_world;
        world_vertices.push_back(v_world);
    }
    return world_vertices;
}

inline std::vector<glm::vec2> getEgoRightVertices(const glm::vec2& position,
                                      double heading_angle_degree,
                                      const glm::vec2& size,          // (length, width)
                                      const glm::vec2& center_offset) // (x, y)
{
    double length = size.x;
    double width = size.y;
    double dx = length / 2.0;
    double dy = width / 2.0;

    // Local rectangle corners (FR, RR)
    std::vector<glm::vec2> local_vertices = {
        { dx,  dy},
        {-dx,  dy} 
    };

    // Rotation matrix (counter-clockwise)
    // remember to change sign due to different convention
    double theta = -glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
             sin(theta),  cos(theta));

    // Center in world coordinates
    glm::vec2 center_world = getCenterRect(position, heading_angle_degree, center_offset);

    // Rotate and translate each vertex
    std::vector<glm::vec2> world_vertices;
    world_vertices.reserve(local_vertices.size());

    for (const auto& v_local : local_vertices) {
        glm::vec2 v_world = rot * v_local + center_world;
        world_vertices.push_back(v_world);
    }
    return world_vertices;
}

inline std::vector<glm::vec2> getObjectWorldVertices(const glm::vec2& position, float heading_angle_degree, const std::vector<glm::vec2>& local_vertices) {
    // remember to change sign due to different convention
    float theta = -glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
                  sin(theta),  cos(theta));
    std::vector<glm::vec2> world_vertices;
    world_vertices.reserve(local_vertices.size());
    for (const auto& v_local : local_vertices) {
        glm::vec2 v_world = rot * v_local + position;
        world_vertices.push_back(v_world);
    }
    return world_vertices;
}

inline bool isCollision(const std::vector<glm::vec2>& vertices1, const std::vector<glm::vec2>& vertices2) {
    auto projectOntoAxis = [](const std::vector<glm::vec2>& verts, const glm::vec2& axis) {
        float minP = std::numeric_limits<float>::infinity();
        float maxP = -std::numeric_limits<float>::infinity();
        for (const auto &v : verts) {
            float p = glm::dot(v, axis);
            if (p < minP) minP = p;
            if (p > maxP) maxP = p;
        }
        return std::pair<float,float>(minP, maxP);
    };

    auto overlaps = [](float minA, float maxA, float minB, float maxB, float eps = 1e-6f) {
        // If intervals [minA, maxA] and [minB, maxB] do NOT overlap, return false
        return !(maxA < minB - eps || maxB < minA - eps);
    };

    // Helper to check all axes from polygon A and B
    auto checkAxesFrom = [&](const std::vector<glm::vec2>& polyA, const std::vector<glm::vec2>& polyB) {
        size_t n = polyA.size();
        for (size_t i = 0; i < n; ++i) {
            glm::vec2 p1 = polyA[i];
            glm::vec2 p2 = polyA[(i + 1) % n];
            glm::vec2 edge = p2 - p1;
            // perpendicular axis (no need to normalize; projection scales equally)
            glm::vec2 axis(-edge.y, edge.x);

            // If axis is (0,0) (degenerate edge), skip it
            if (glm::length2(axis) < 1e-12f) continue;

            auto [minA, maxA] = projectOntoAxis(polyA, axis);
            auto [minB, maxB] = projectOntoAxis(polyB, axis);

            if (!overlaps(minA, maxA, minB, maxB)) {
                // Found a separating axis -> no collision
                return false;
            }
        }
        return true;
    };

    // Check axes from both polygons
    if (!checkAxesFrom(vertices1, vertices2)) return false;
    if (!checkAxesFrom(vertices2, vertices1)) return false;

    // No separating axis found -> polygons intersect (or touch)
    return true;
}

inline bool startsWith(const std::string& text, const std::string& prefix) {
    return text.find(prefix) == 0;
}

#endif