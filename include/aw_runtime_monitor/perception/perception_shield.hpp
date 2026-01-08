#ifndef PERCEPTION_SHIELD_HPP
#define PERCEPTION_SHIELD_HPP

#include <string>
#include <optional>
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tqtl/tqtl.hpp>

class PerceptionShield
{
public:
    PerceptionShield(std::string spec_formula_str="T", size_t window_size=10, double speed_threshold_activation=3.0) :
            logger_(rclcpp::get_logger("perception_shield")) {
        // parse the perception spec formula
        this->perception_spec_ = tqtl::Parser::parse(spec_formula_str);
        this->window_size_ = window_size;
        this->perp_data_stream_ = tqtl::DataStream();
        this->speed_threshold_activation_ = speed_threshold_activation;
    }

    std::optional<autoware_perception_msgs::msg::PredictedObjects>
    verify(const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg, nlohmann::json& recorded_data);

private:
    tqtl::FormulaPtr perception_spec_;
    size_t window_size_ = 10;
    tqtl::DataStream perp_data_stream_;
    double speed_threshold_activation_ = 3.0; // m/s

    rclcpp::Logger logger_;
};

#endif