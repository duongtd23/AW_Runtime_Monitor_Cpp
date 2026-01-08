#include "aw_runtime_monitor/utils.hpp"
#include <cmath>
#include <sstream> // for std::istringstream (read each line)
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"

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
// heading_angle_degree in counter-clockwise degree
glm::vec2 getCenterRect(const glm::vec2& position, double heading_angle_degree, const glm::vec2& center_offset) {
    // need to change sign because of different convention
    double theta = -glm::radians(heading_angle_degree);
    glm::mat2 rot(cos(theta), -sin(theta),
             sin(theta),  cos(theta));
    return position + rot * center_offset;
}

std::vector<glm::vec2> getEgoWorldVertices(const glm::vec2& position,
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

std::vector<glm::vec2> getObjectWorldVertices(const glm::vec2& position, float heading_angle_degree, const std::vector<glm::vec2>& local_vertices) {
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
    // size_t pos = mainString.find(oldSubstring);
    // if (pos != std::string::npos) {
    //     mainString.replace(pos, oldSubstring.length(), newSubstring);
    // }
    size_t pos = 0;
    while ((pos = mainString.find(oldSubstring, pos)) != std::string::npos) {
        mainString.replace(pos, oldSubstring.length(), newSubstring);
        pos += newSubstring.length();
    }
}

// Helper: trim leading/trailing whitespace
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Extract result from Maude output of red command
// Extract the value after "result:" from the given text
std::string extractResultFromOutput(const std::string& text) {
    std::istringstream iss(text);
    std::string line;
    std::string result="";
    bool in_result_block = false;

    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);

        if (!in_result_block) {
            // Detect start of "result ..." line
            if (trimmed.rfind("result", 0) == 0) {  // starts with "result"
                size_t colon_pos = trimmed.find(':');
                if (colon_pos != std::string::npos) {
                    // Extract text after colon
                    result = trim(trimmed.substr(colon_pos + 1));
                    in_result_block = true;
                }
            }
        } else {
            // Continue collecting if the line seems like a continuation
            if (trimmed.empty()) break; // blank line => stop
            if (std::isspace(line[0])) {
                // Indented line, so it's part of the result
                result += " " + trim(line);
            } else {
                // Non-indented and not blank: probably next section
                break;
            }
        }
    }
    // if no "result" line found, indicating a syntax error of the safety formula
    return result;
}

std::vector<std::string> toVectorString(const std::string& input) {
    // Pattern to find either a quoted string (e.g., `"token"`) or
    // a non-space token (for handling potential unquoted tokens).
    // The pattern captures the content inside the quotes.
    std::regex pattern(R"("[^"]+"|[^ ]+)");
    std::vector<std::string> tokens;

    for (std::sregex_iterator it(input.begin(), input.end(), pattern), end; it != end; ++it) {
        // The match is the full token including quotes.
        std::string token = it->str();

        // If the token starts and ends with a quote, remove them.
        if (token.front() == '\"' && token.back() == '\"') {
            token = token.substr(1, token.length() - 2);
        }
        tokens.push_back(token);
    }

    // // Print the results
    // for (const auto& token : tokens) {
    //     std::cout << "Token: " << token << std::endl;
    // }

    return tokens;
}

glm::vec3 jsonPointToVector3(const nlohmann::json& dict_point) {
    float x = dict_point["x"];
    float y = dict_point["y"];
    float z = dict_point["z"];
    return glm::vec3(x, y, z);
}


glm::vec3 getCurrentVelocity(const nlohmann::json& recorded_messages) {
    if (recorded_messages.find(EstimatedKinematicTopic::TRACE_KEY()) == recorded_messages.end() ||
        recorded_messages.at(EstimatedKinematicTopic::TRACE_KEY()).empty()) {
            return glm::vec3(0.0f);
    }
    auto kin = recorded_messages[EstimatedKinematicTopic::TRACE_KEY()].back();
    return jsonPointToVector3(kin["twist"]["linear"]);
}

float getCurrentSpeed(const nlohmann::json& recorded_messages) {
    return glm::length(getCurrentVelocity(recorded_messages));
}

glm::vec3 rosPointToVector3(const geometry_msgs::msg::Point& point_msg) {
    return glm::vec3(point_msg.x, point_msg.y, point_msg.z);
}
glm::vec3 rosPointToVector3(const geometry_msgs::msg::Vector3& vector_msg) {
    return glm::vec3(vector_msg.x, vector_msg.y, vector_msg.z);
}