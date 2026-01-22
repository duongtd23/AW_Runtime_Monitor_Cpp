#ifndef PERCEPTION_SHIELD_HPP
#define PERCEPTION_SHIELD_HPP

#include <string>
#include <optional>
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tqtl/tqtl.hpp>
#include <vector>

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

    tqtl::FormulaPtr getSpecFormula() const {
        return perception_spec_;
    }

    void cacheRecordedMsg(const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg) {
        recorded_perp_msgs_.emplace_back(perp_obj_msg);
        // keep only up to window_size_ msgs
        while (recorded_perp_msgs_.size() > window_size_ + 5) {
            recorded_perp_msgs_.erase(recorded_perp_msgs_.begin());
        }
    }

    void reset() {
        perp_data_stream_.clear();
        recorded_perp_msgs_.clear();
    }
private:
    tqtl::FormulaPtr perception_spec_;
    size_t window_size_ = 10;
    tqtl::DataStream perp_data_stream_;
    double speed_threshold_activation_ = 3.0; // m/s
    // cache recorded msgs
    std::vector<autoware_perception_msgs::msg::PredictedObjects> recorded_perp_msgs_;

    rclcpp::Logger logger_;
};

#endif