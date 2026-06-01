#ifndef PERCEPTION_SHIELD_HPP
#define PERCEPTION_SHIELD_HPP

#include <string>
#include <optional>
#include "aw_runtime_monitor/perception/perception_object.hpp"
#include "aw_runtime_monitor/localization/estimated_kinematic.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tqtl/tqtl.hpp>
#include <vector>
#include <algorithm> // for std::remove_if

class PerceptionShield
{
public:
    struct VerificationResult {
        bool is_safe; // true iff the original perception message satisfies the specification
        bool is_revised; // true iff the original message is revised
        autoware_perception_msgs::msg::PredictedObjects revised_msg; //non-sense unless is_revised == true
    };
    struct DroppedObjectPrediction {
        double timestamp;
        autoware_perception_msgs::msg::PredictedObject predicted_object;
        size_t no_frames_fixed;
    };
    PerceptionShield(std::string spec_formula_str="T", size_t window_size=10, 
            double speed_threshold_activation=3.0, size_t max_prediction_frames=3, bool enable_revision=true) :
            logger_(rclcpp::get_logger("perception_shield")) {
        // parse the perception spec formula
        this->perception_spec_ = tqtl::Parser::parse(spec_formula_str);
        this->window_size_ = window_size;
        this->perp_data_stream_ = tqtl::DataStream();
        this->speed_threshold_activation_ = speed_threshold_activation;
        this->max_prediction_frames_ = max_prediction_frames;
        this->enable_revision_ = enable_revision;
    }

    VerificationResult verify(const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg, const nlohmann::json& recorded_data);

    tqtl::FormulaPtr getSpecFormula() const {
        return perception_spec_;
    }

    void cacheRecordedMsg(const autoware_perception_msgs::msg::PredictedObjects& perp_obj_msg) {
        recorded_perp_msgs_.emplace_back(perp_obj_msg);
        // keep only up to window_size_ msgs
        while (recorded_perp_msgs_.size() > window_size_) {
            recorded_perp_msgs_.erase(recorded_perp_msgs_.begin());
        }
    }

    void reset() {
        perp_data_stream_.clear();
        recorded_perp_msgs_.clear();
        dropped_object_predictions_.clear();
        // Force deallocation
        std::vector<autoware_perception_msgs::msg::PredictedObjects>().swap(recorded_perp_msgs_);
        std::vector<DroppedObjectPrediction>().swap(dropped_object_predictions_);
    }
private:
    bool enable_revision_ = true;

    // dynamically change
    tqtl::DataStream perp_data_stream_;
    std::vector<autoware_perception_msgs::msg::PredictedObjects> recorded_perp_msgs_;
    // for each detected dropped object, we save the prediction information from the first frame when it was dropped
    // this is used to continue predicting the dropped object for a few (precisely, max_prediction_frames_ - 1) frames afterwards
    std::vector<DroppedObjectPrediction> dropped_object_predictions_;
    size_t max_prediction_frames_ = 3; // max number of frames to keep predicting a dropped object

    // fixed per run
    tqtl::FormulaPtr perception_spec_;
    // keep only up to window_size_ perception msgs
    size_t window_size_ = 10;
    double speed_threshold_activation_ = 3.0; // m/s

    rclcpp::Logger logger_;
};

#endif