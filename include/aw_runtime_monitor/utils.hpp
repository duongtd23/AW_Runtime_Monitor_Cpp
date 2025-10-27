#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp" // For JSON functionality
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <glm/glm.hpp>  // for vector computation
#include <glm/gtx/norm.hpp> 
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <vector>
#include <cmath>
#include <regex>

double roundDouble(double original_value);
nlohmann::json vector3ToJsonPoint(const geometry_msgs::msg::Vector3& obj, bool round=true);
nlohmann::json pointToJsonPoint(const geometry_msgs::msg::Point& msg, bool round=true);
nlohmann::json pointToJsonPoint(const geometry_msgs::msg::Point32& msg, bool round=true);
nlohmann::json quaternionToJsonEulerAngles(const geometry_msgs::msg::Quaternion& quat_msg, bool round=true);
double timestamp(const builtin_interfaces::msg::Duration& duration, bool round=true);
double timestamp(const builtin_interfaces::msg::Time& stamp, bool round=true);
double timestamp(const std_msgs::msg::Header& header, bool round=true);
std::string uuidstr(const std::array<unsigned char, 16>& uuid);

glm::vec2 jsonPointToVector2(const nlohmann::json& dict_point);

glm::vec2 getCenterRect(const glm::vec2& position, double heading_angle_degree, const glm::vec2& center_offset);
std::vector<glm::vec2> getEgoWorldVertices(const glm::vec2& position,
                                         double heading_angle_degree,
                                         const glm::vec2& size,           // (length, width)
                                         const glm::vec2& center_offset); // (x, y)

std::vector<glm::vec2> getObjectWorldVertices(const glm::vec2& position, float heading_angle_degree, const std::vector<glm::vec2>& local_vertices);

bool isCollision(const std::vector<glm::vec2>& vertices1, const std::vector<glm::vec2>& vertices2);

bool startsWith(const std::string& text, const std::string& prefix);
std::vector<std::string> splitString(const std::string& input, char delimiter);
void replaceSubStr(std::string& mainString, const std::string& oldSubstring, const std::string& newSubstring);

std::string trim(const std::string& s);
std::string extractResultFromOutput(const std::string& text);
std::vector<std::string> toVectorString(const std::string& result);

#endif
