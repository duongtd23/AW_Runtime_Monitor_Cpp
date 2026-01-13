#ifndef KALMAN_OBJECT_PREDICTION_HPP
#define KALMAN_OBJECT_PREDICTION_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "autoware_perception_msgs/msg/predicted_object.hpp"

/**
 * @brief Kalman Filter-based object prediction for perception shielding.
 * 
 * This class implements the algorithm described in the paper:
 * - State representation: x = [p; v] ∈ ℝ⁶
 * - Constant velocity motion model with white noise acceleration
 * - Velocity blending from reported and position-derived sources
 * - Adaptive velocity covariance estimation
 * - History-based state fusion using recursive Kalman filtering
 */
class KalmanObjectPrediction {
public:
    struct ObjectHistoryState {
        double timestamp;
        autoware_perception_msgs::msg::PredictedObject raw_object;
    };
    using ObjectHistory = std::vector<ObjectHistoryState>;

    /**
     * @brief Configuration parameters for the predictor
     */
    struct Config {
        double alpha = 0.5;                 // Velocity blending weight (0 = use reported, 1 = use derived)
        double min_velocity_var = 0.1;      // Minimum velocity variance
        double max_velocity_var = 10.0;     // Maximum velocity variance
        double min_acceleration_var = 0.1;  // Minimum acceleration variance for process noise
        double max_acceleration_var = 4.0;  // Maximum acceleration variance for process noise
    };

    KalmanObjectPrediction() : config_(Config{}) {}
    explicit KalmanObjectPrediction(const Config& config) : config_(config) {}

    /**
     * @brief Predict the state of a dropped object at a target timestamp.
     * 
     * Implements the algorithm from Section X of the paper:
     * 1. Initialize state from first observation (Section X.4.1)
     * 2. Recursive prediction-update through history (Section X.4.2)
     * 3. Final prediction for missing frame (Section X.5)
     * 
     * @param object_id The ID of the dropped object
     * @param history Vector of past observations (oldest first)
     * @param target_timestamp The time at which to predict the object state
     * @return The predicted object at target_timestamp
     */
    autoware_perception_msgs::msg::PredictedObject predictDroppedObject(
            const std::string& object_id,
            const ObjectHistory& history,
            double target_timestamp) {
        
        if (history.empty()) {
            throw std::runtime_error("Cannot predict with empty history for object: " + object_id);
        }

        if (history.size() == 1) {
            // Only one observation: use simple extrapolation
            return predictFromSingleObservation(history[0], target_timestamp);
        }

        // ========================================
        // Step 1: Estimate velocity noise from history (Section X.3.3)
        // ========================================
        double sigma_v_squared = estimateVelocityVariance(history); // TODO
        double sigma_a_squared = std::clamp(
            sigma_v_squared / std::pow(history[1].timestamp - history[0].timestamp, 2),
            config_.min_acceleration_var,
            config_.max_acceleration_var
        );

        // ========================================
        // Step 2: Initialize state from first observation (Section X.4.1)
        // ========================================
        Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(6);  // State estimate
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6, 6);   // State covariance

        const auto& first_obs = history[0];
        auto [p0, v0_reported, R_p0, R_v0_reported] = extractObservation(first_obs);

        // Blended velocity for initialization (no derived velocity available yet)
        Eigen::Vector3d v0_blended = v0_reported;

        // Initial state: x_{k-m+1} = [p_{k-m+1}; v̂_{k-m+1}]
        x_hat.head<3>() = p0;
        x_hat.tail<3>() = v0_blended;

        // Initial covariance with estimated velocity variance
        Eigen::Matrix3d R_v0 = computeVelocityCovariance(R_v0_reported, sigma_v_squared);
        P.block<3, 3>(0, 0) = R_p0;
        P.block<3, 3>(3, 3) = R_v0;

        // ========================================
        // Step 3: Recursive Prediction-Update (Section X.4.2)
        // ========================================
        for (size_t i = 1; i < history.size(); ++i) {
            const auto& prev_obs = history[i - 1];
            const auto& curr_obs = history[i];

            double dt = curr_obs.timestamp - prev_obs.timestamp;
            if (dt <= 0) {
                continue; // Skip invalid time steps
            }

            // Extract current observation
            auto [p_i, v_i_reported, R_p_i, R_v_i_reported] = extractObservation(curr_obs);
            auto [p_prev, v_prev_reported, R_p_prev, R_v_prev_reported] = extractObservation(prev_obs);

            // Compute position-derived velocity (Section X.3.2)
            Eigen::Vector3d v_tilde = (p_i - p_prev) / dt;

            // Blended velocity estimate
            Eigen::Vector3d v_hat = (1.0 - config_.alpha) * v_i_reported + config_.alpha * v_tilde;

            // Velocity covariance (Section X.3.3)
            Eigen::Matrix3d R_v_i = computeVelocityCovariance(R_v_i_reported, sigma_v_squared);

            // Construct measurement vector z_i = [p_i; v̂_i]
            Eigen::VectorXd z_i(6);
            z_i.head<3>() = p_i;
            z_i.tail<3>() = v_hat;

            // Measurement covariance R_i
            Eigen::MatrixXd R_i = Eigen::MatrixXd::Zero(6, 6);
            R_i.block<3, 3>(0, 0) = R_p_i;
            R_i.block<3, 3>(3, 3) = R_v_i;

            // --- Prediction Step ---
            Eigen::MatrixXd F = stateTransitionMatrix(dt);
            Eigen::MatrixXd Q = processNoiseCovariance(dt, sigma_a_squared); // TODO: May update sigma_v_squared here with a fixed value

            Eigen::VectorXd x_pred = F * x_hat;
            Eigen::MatrixXd P_pred = F * P * F.transpose() + Q;

            // --- Innovation Step ---
            Eigen::VectorXd y = z_i - x_pred;
            Eigen::MatrixXd S = P_pred + R_i;

            // --- Kalman Gain ---
            Eigen::MatrixXd K = P_pred * S.inverse();

            // --- Update Step ---
            x_hat = x_pred + K * y;
            P = (Eigen::MatrixXd::Identity(6, 6) - K) * P_pred;
        }

        // ========================================
        // Step 4: Final Prediction for Missing Frame (Section X.5)
        // ========================================
        double dt_final = target_timestamp - history.back().timestamp;
        
        Eigen::MatrixXd F_final = stateTransitionMatrix(dt_final);
        Eigen::MatrixXd Q_final = processNoiseCovariance(dt_final, sigma_a_squared);

        Eigen::VectorXd x_predicted = F_final * x_hat;
        Eigen::MatrixXd P_predicted = F_final * P * F_final.transpose() + Q_final;

        // ========================================
        // Step 5: Build output message
        // ========================================
        return buildPredictedObject(
            history.back().raw_object,
            x_predicted,
            P_predicted,
            sigma_v_squared,
            dt_final
        );
    }

private:
    Config config_;

    /**
     * @brief Extract observation data from a history state
     * @return Tuple of (position, velocity, position_covariance, velocity_covariance)
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d>
    extractObservation(const ObjectHistoryState& obs) {
        const auto& pose = obs.raw_object.kinematics.initial_pose_with_covariance;
        const auto& twist = obs.raw_object.kinematics.initial_twist_with_covariance;

        // Position
        Eigen::Vector3d p(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        );

        // Velocity
        Eigen::Vector3d v(
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z
        );

        // Position covariance (extract diagonal from 6x6)
        Eigen::Matrix3d R_p = Eigen::Matrix3d::Zero();
        R_p(0, 0) = std::max(0.01, pose.covariance[0]);   // xx
        R_p(1, 1) = std::max(0.01, pose.covariance[7]);   // yy
        R_p(2, 2) = std::max(0.01, pose.covariance[14]);  // zz

        // Velocity covariance (extract diagonal from 6x6)
        Eigen::Matrix3d R_v = Eigen::Matrix3d::Zero();
        R_v(0, 0) = std::max(0.01, twist.covariance[0]);   // xx
        R_v(1, 1) = std::max(0.01, twist.covariance[7]);   // yy
        R_v(2, 2) = std::max(0.01, twist.covariance[14]);  // zz

        return {p, v, R_p, R_v};
    }

    /**
     * @brief Estimate velocity variance from history (Section X.3.3)
     * 
     * Computes: σ²_{v,est} = (1/n) Σ ||e_i||²
     * where e_i = ṽ_i - v_{i-1}
     */
    double estimateVelocityVariance(const ObjectHistory& history) {
        if (history.size() < 2) {
            return 1.0; // Default variance
        }

        double error_sum = 0.0;
        int count = 0;

        for (size_t i = 1; i < history.size(); ++i) {
            double dt = history[i].timestamp - history[i - 1].timestamp;
            if (dt <= 0) continue;

            auto [p_i, v_i, R_p_i, R_v_i] = extractObservation(history[i]);
            auto [p_prev, v_prev, R_p_prev, R_v_prev] = extractObservation(history[i - 1]);

            // Position-derived velocity
            Eigen::Vector3d v_tilde = (p_i - p_prev) / dt;

            // Velocity error: e_i = ṽ_i - v_{i-1}
            Eigen::Vector3d e = v_tilde - v_prev;

            error_sum += e.squaredNorm();
            count++;
        }

        if (count == 0) {
            return 1.0;
        }

        double sigma_v_squared = error_sum / count;

        // Clamp to reasonable range
        return std::clamp(sigma_v_squared, config_.min_velocity_var, config_.max_velocity_var);
    }

    /**
     * @brief Compute velocity covariance with conservative estimate (Section X.3.3)
     * 
     * R^{(v)}_i = max(R^{(v)}_{i,rep}, σ²_{v,est} · I_3)
     */
    Eigen::Matrix3d computeVelocityCovariance(
            const Eigen::Matrix3d& R_v_reported,
            double sigma_v_squared) {
        
        Eigen::Matrix3d R_v = Eigen::Matrix3d::Zero();
        
        // Element-wise max of diagonals
        R_v(0, 0) = std::max(R_v_reported(0, 0), sigma_v_squared);
        R_v(1, 1) = std::max(R_v_reported(1, 1), sigma_v_squared);
        R_v(2, 2) = std::max(R_v_reported(2, 2), sigma_v_squared);

        return R_v;
    }

    /**
     * @brief State transition matrix F(Δt) (Section X.2)
     * 
     * F(Δt) = [ I_3    Δt·I_3 ]
     *         [ 0_3      I_3  ]
     */
    Eigen::MatrixXd stateTransitionMatrix(double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
        return F;
    }

    /**
     * @brief Process noise covariance Q(Δt) (Section X.2)
     * 
     * Based on white noise acceleration model:
     * Q(Δt) = σ_a² · [ (Δt⁴/4)·I_3    (Δt³/2)·I_3 ]
     *               [ (Δt³/2)·I_3      Δt²·I_3    ]
     */
    Eigen::MatrixXd processNoiseCovariance(double dt, double sigma_a_squared) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        // Position-position block
        Q.block<3, 3>(0, 0) = (dt4 / 4.0) * sigma_a_squared * Eigen::Matrix3d::Identity();

        // Position-velocity and velocity-position blocks
        Q.block<3, 3>(0, 3) = (dt3 / 2.0) * sigma_a_squared * Eigen::Matrix3d::Identity();
        Q.block<3, 3>(3, 0) = (dt3 / 2.0) * sigma_a_squared * Eigen::Matrix3d::Identity();

        // Velocity-velocity block
        Q.block<3, 3>(3, 3) = dt2 * sigma_a_squared * Eigen::Matrix3d::Identity();

        return Q;
    }

    /**
     * @brief Simple extrapolation when only one observation is available
     */
    autoware_perception_msgs::msg::PredictedObject predictFromSingleObservation(
            const ObjectHistoryState& obs,
            double target_timestamp) {
        
        auto predicted = obs.raw_object;
        double dt = target_timestamp - obs.timestamp;

        auto& pose = predicted.kinematics.initial_pose_with_covariance;
        const auto& twist = predicted.kinematics.initial_twist_with_covariance;

        // Simple linear extrapolation: p_{k+1} = p_k + v_k · Δt
        pose.pose.position.x += twist.twist.linear.x * dt;
        pose.pose.position.y += twist.twist.linear.y * dt;
        pose.pose.position.z += twist.twist.linear.z * dt;

        // Increase uncertainty
        double uncertainty_growth = 1.0 * dt * dt;
        pose.covariance[0] += uncertainty_growth;
        pose.covariance[7] += uncertainty_growth;
        pose.covariance[14] += uncertainty_growth;

        return predicted;
    }

    /**
     * @brief Update the predicted path based on the final predicted state.
     * 
     * @param original_path The original predicted path to update
     */
    void updatePredictedPath(autoware_perception_msgs::msg::PredictedPath& original_path,
                             const Eigen::VectorXd& updated_position) {
        if (original_path.path.empty())
            return;
        
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        for (size_t i = 0; i < original_path.path.size(); ++i) {
            const auto& point = original_path.path[i].position;
            Eigen::Vector3d p_point(point.x, point.y, point.z);
            double dist = (p_point - updated_position.head<3>()).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        original_path.path.erase(original_path.path.begin(), original_path.path.begin() + closest_idx);
        if (original_path.path.size() >= 2) {
            const auto& first_point = original_path.path[0].position;
            const auto& second_point = original_path.path[1].position;
            Eigen::Vector3d p_first(first_point.x, first_point.y, first_point.z);
            Eigen::Vector3d p_second(second_point.x, second_point.y, second_point.z);
            Eigen::Vector3d dir = (p_second - p_first).normalized();
            Eigen::Vector3d to_updated = (updated_position.head<3>() - p_first);
            double projection = to_updated.dot(dir);
            if (projection >= 0) {
                original_path.path.erase(original_path.path.begin());
            }
        }
        geometry_msgs::msg::Pose current_pose;
        current_pose.position.x = updated_position(0);
        current_pose.position.y = updated_position(1);
        current_pose.position.z = updated_position(2);
        current_pose.orientation = original_path.path[0].orientation;
        original_path.path.insert(original_path.path.begin(), current_pose);
    }

    /**
     * @brief Build the output PredictedObject message
     */
    autoware_perception_msgs::msg::PredictedObject buildPredictedObject(
            const autoware_perception_msgs::msg::PredictedObject& template_obj,
            const Eigen::VectorXd& x_predicted,
            const Eigen::MatrixXd& P_predicted,
            double sigma_v_squared,
            double /*dt*/) {
        
        auto predicted = template_obj;

        // Extract predicted position: p̂_{k+1} = x_predicted[0:3]
        auto& pose = predicted.kinematics.initial_pose_with_covariance;
        pose.pose.position.x = x_predicted(0);
        pose.pose.position.y = x_predicted(1);
        pose.pose.position.z = x_predicted(2);

        // Extract predicted velocity: v̂_{k+1} = x_predicted[3:6]
        auto& twist = predicted.kinematics.initial_twist_with_covariance;
        twist.twist.linear.x = x_predicted(3);
        twist.twist.linear.y = x_predicted(4);
        twist.twist.linear.z = x_predicted(5);

        // Update position covariance from P_predicted
        pose.covariance[0] = P_predicted(0, 0);   // xx
        pose.covariance[7] = P_predicted(1, 1);   // yy
        pose.covariance[14] = P_predicted(2, 2);  // zz

        // Velocity covariance: R^{(v)}_{k+1} = max(P^{(vv)}_{k+1}, σ²_{v,est}·I_3)
        twist.covariance[0] = std::max(P_predicted(3, 3), sigma_v_squared);   // xx
        twist.covariance[7] = std::max(P_predicted(4, 4), sigma_v_squared);   // yy
        twist.covariance[14] = std::max(P_predicted(5, 5), sigma_v_squared);  // zz

        // update predicted travel paths
        for (auto& path : predicted.kinematics.predicted_paths) {
            updatePredictedPath(path, x_predicted);
        }
        return predicted;
    }
};

#endif // KALMAN_OBJECT_PREDICTION_HPP