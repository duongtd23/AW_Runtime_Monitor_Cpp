#ifndef AW_UTILS_HPP
#define AW_UTILS_HPP

#include <iostream>
#include <string>
#include <utils.hpp>
#include "nlohmann/json.hpp" // For JSON functionality
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <regex>

inline double roundDouble(double original_value) {
    return std::round(original_value * 1000.0) / 1000.0;
}
inline nlohmann::json vector3ToJsonPoint(const geometry_msgs::msg::Vector3& obj, bool round=true) {
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

inline nlohmann::json pointToJsonPoint(const geometry_msgs::msg::Point& msg, bool round=true) {
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
inline nlohmann::json pointToJsonPoint(const geometry_msgs::msg::Point32& msg, bool round=true) {
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

/**
 * @brief Convert a ROS Vector3 message representing angular velocity (in radians/sec)
 *        to a JSON object containing roll, pitch, yaw in degrees.
 */
inline nlohmann::json rosAngularVelToJsonPoint(const geometry_msgs::msg::Vector3& angular_vel_msg, bool round=true) {
    nlohmann::json j;
    if (round) {
        j["roll"] = roundDouble(angular_vel_msg.x * 180/M_PI);
        j["pitch"] = roundDouble(angular_vel_msg.y * 180/M_PI);
        j["yaw"] = roundDouble(angular_vel_msg.z * 180/M_PI);
    }
    else {
        j["roll"] = angular_vel_msg.x * 180/M_PI;
        j["pitch"] = angular_vel_msg.y * 180/M_PI;
        j["yaw"] = angular_vel_msg.z * 180/M_PI;
    }
    return j;
}

/**
 * @brief Convert a ROS Quaternion message to Euler angles (roll, pitch, yaw) in degrees.
 */
inline std::array<double, 3> quaternionToEulerAngles(const geometry_msgs::msg::Quaternion& quat_msg) {
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

/**
 * @brief Convert a ROS Quaternion message to a JSON object containing Euler angles (roll, pitch, yaw) in degrees.
 */
inline nlohmann::json quaternionToJsonEulerAngles(const geometry_msgs::msg::Quaternion& quat_msg, bool round=true) {
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

inline double timestamp(const builtin_interfaces::msg::Duration& duration, bool round=true) {
    double re = duration.sec + duration.nanosec / 1e9;
    return round ? roundDouble(re) : re;
}
inline double timestamp(const builtin_interfaces::msg::Time& stamp, bool round=true) {
    double re = stamp.sec + stamp.nanosec / 1e9;
    return round ? roundDouble(re) : re;
}
inline double timestamp(const std_msgs::msg::Header& header, bool round=true) {
    return timestamp(header.stamp, round);
}

inline std::string uuidstr(const std::array<unsigned char, 16>& uuid) {
    std::string result_str;
    for (size_t i = 0; i < uuid.size(); ++i) {
        result_str += std::to_string(static_cast<int>(uuid[i]));
        if (i < uuid.size() - 1) {
            result_str += " ";
        }
    }
    return result_str;
}

inline glm::vec2 jsonPointToVector2(const nlohmann::json& dict_point) {
    float x = dict_point["x"];
    float y = dict_point["y"];
    return glm::vec2(x, y);
}
inline glm::vec3 jsonPointToVector3(const nlohmann::json& dict_point) {
    float x = dict_point["x"];
    float y = dict_point["y"];
    float z = dict_point["z"];
    return glm::vec3(x, y, z);
}

inline std::vector<std::string> splitString(const std::string& input, char delimiter) {
    std::vector<std::string> result;
    std::istringstream iss(input); 
    std::string token;

    while (std::getline(iss, token, delimiter)) {
        result.push_back(token);
    }
    return result;
}

inline void replaceSubStr(std::string& mainString, const std::string& oldSubstring, const std::string& newSubstring) {
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
inline std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Extract result from Maude output of red command
// Extract the value after "result:" from the given text
inline std::string extractResultFromOutput(const std::string& text) {
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

inline std::vector<std::string> toVectorString(const std::string& input) {
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

inline glm::vec3 rosPointToVector3(const geometry_msgs::msg::Point& point_msg) {
    return glm::vec3(point_msg.x, point_msg.y, point_msg.z);
}
inline glm::vec3 rosPointToVector3(const geometry_msgs::msg::Vector3& vector_msg) {
    return glm::vec3(vector_msg.x, vector_msg.y, vector_msg.z);
}

#endif
