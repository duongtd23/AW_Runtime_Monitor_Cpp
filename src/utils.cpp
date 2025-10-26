#include "aw_runtime_monitor/utils.hpp"
#include <cmath>

double roundDouble(double original_value) {
    return std::round(original_value * 1000.0) / 1000.0;
}

nlohmann::json vector3ToJsonPoint(const geometry_msgs::msg::Vector3& obj, bool round) {
    nlohmann::json j;
    if (round) {
        j["x"] = roundDouble(obj.x);
        j["y"] = roundDouble(obj.y);
        j["z"] = roundDouble(obj.z);
    }
    else {
        j["x"] = obj.x;
        j["y"] = obj.y;
        j["z"] = obj.z;
    }
    return j;
}

nlohmann::json pointToJsonPoint(const geometry_msgs::msg::Point& msg, bool round) {
    nlohmann::json j;
    if (round) {
        j["x"] = roundDouble(msg.x);
        j["y"] = roundDouble(msg.y);
        j["z"] = roundDouble(msg.z);
    }
    else {
        j["x"] = msg.x;
        j["y"] = msg.y;
        j["z"] = msg.z;
    }
    return j;
}
nlohmann::json pointToJsonPoint(const geometry_msgs::msg::Point32& msg, bool round) {
    nlohmann::json j;
    if (round) {
        j["x"] = roundDouble(msg.x);
        j["y"] = roundDouble(msg.y);
        j["z"] = roundDouble(msg.z);
    }
    else {
        j["x"] = msg.x;
        j["y"] = msg.y;
        j["z"] = msg.z;
    }
    return j;
}

std::array<double, 3> quaternionToEulerAngles(const geometry_msgs::msg::Quaternion& quat_msg) {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat_msg, tf_quat);

    tf2::Matrix3x3 m(tf_quat);

    // Extract roll, pitch, and yaw from the matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;
    return {roll, pitch, yaw};
}

nlohmann::json quaternionToJsonEulerAngles(const geometry_msgs::msg::Quaternion& quat_msg, bool round) {
    auto angles = quaternionToEulerAngles(quat_msg);
    nlohmann::json j;
    if (round){
        j["x"] = roundDouble(angles[0]);
        j["y"] = roundDouble(angles[1]);
        j["z"] = roundDouble(angles[2]);
    }
    else {
        j["x"] = (angles[0]);
        j["y"] = (angles[1]);
        j["z"] = (angles[2]);
    }
    return j;
}

double timestamp(const builtin_interfaces::msg::Duration& duration, bool round) {
    double re = duration.sec + duration.nanosec / 1e9;
    return round ? roundDouble(re) : re;
}
double timestamp(const builtin_interfaces::msg::Time& stamp, bool round) {
    double re = stamp.sec + stamp.nanosec / 1e9;
    return round ? roundDouble(re) : re;
}
double timestamp(const std_msgs::msg::Header& header, bool round) {
    return timestamp(header.stamp, round);
}

std::string uuidstr(const std::array<unsigned char, 16>& uuid) {
    std::string result_str;
    for (size_t i = 0; i < uuid.size(); ++i) {
        result_str += std::to_string(static_cast<int>(uuid[i]));
        if (i < uuid.size() - 1) {
            result_str += " ";
        }
    }
    return result_str;
}

glm::vec2 jsonPointToVector2(const nlohmann::json& dict_point) {
    float x = dict_point["x"];
    float y = dict_point["y"];
    return glm::vec2(x, y);
}

// Return the center point of the vehicle in world coordinates.
glm::vec2 getCenterRect(const glm::vec2& position, double heading_angle_degree, const glm::vec2& center_offset) {
    double theta = glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
             sin(theta),  cos(theta));
    return position + rot * center_offset;
}

std::vector<glm::vec2> getEgoWorldVertices(const glm::vec2& position,
                                         double heading_angle_degree,
                                         const glm::vec2& size,          // (length, width)
                                         const glm::vec2& center_offset) // (x, y)
{
    double width = size.y;
    double length = size.x;
    double dx = width / 2.0;
    double dy = length / 2.0;

    // Local rectangle corners (FR, FL, RL, RR)
    std::vector<glm::vec2> local_vertices = {
        { dx,  dy},  // front-right
        {-dx,  dy},  // front-left
        {-dx, -dy},  // rear-left
        { dx, -dy}   // rear-right
    };

    // Rotation matrix (counter-clockwise)
    double theta = glm::radians(heading_angle_degree);
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

std::vector<glm::vec2> getObjectWorldVertices(const glm::vec2& position, float heading_angle_degree, const std::vector<glm::vec2>& local_vertices) {
    float theta = glm::radians(heading_angle_degree);
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

bool isCollision(const std::vector<glm::vec2>& vertices1, const std::vector<glm::vec2>& vertices2) {
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

bool startsWith(const std::string& text, const std::string& prefix) {
    return text.find(prefix) == 0;
}

std::vector<std::string> splitString(const std::string& input, char delimiter) {
    std::vector<std::string> result;
    std::istringstream iss(input); 
    std::string token;

    while (std::getline(iss, token, delimiter)) {
        result.push_back(token);
    }
    return result;
}

void replaceSubStr(std::string& mainString, const std::string& oldSubstring, const std::string& newSubstring) {
    size_t pos = mainString.find(oldSubstring);
    if (pos != std::string::npos) {
        mainString.replace(pos, oldSubstring.length(), newSubstring);
    }
}