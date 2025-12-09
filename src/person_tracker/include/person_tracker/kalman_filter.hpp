/**
 * @file kalman_filter.hpp
 * @brief Simple 3D Kalman filter for position and velocity estimation
 */

#ifndef PERSON_TRACKER__KALMAN_FILTER_HPP_
#define PERSON_TRACKER__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace person_tracker {

/**
 * @brief Kalman filter for tracking 3D position and velocity
 * State vector: [x, y, z, vx, vy, vz]
 * Measurement: [x, y, z]
 */
class KalmanFilter3D {
public:
    KalmanFilter3D() {
        // State: [x, y, z, vx, vy, vz]
        state_ = Eigen::VectorXd::Zero(6);

        // State covariance matrix (uncertainty)
        P_ = Eigen::MatrixXd::Identity(6, 6);
        P_ *= 1.0;  // Initial uncertainty

        // Process noise covariance
        Q_ = Eigen::MatrixXd::Identity(6, 6);
        Q_.block<3, 3>(0, 0) *= 0.05;  // Position process noise (small - trust motion model)
        Q_.block<3, 3>(3, 3) *= 0.1;   // Velocity process noise (small - velocity changes gradually)

        // Measurement noise covariance
        R_ = Eigen::MatrixXd::Identity(3, 3);
        R_ *= 0.5;  // Measurement noise (higher - detections can be noisy/jittery)

        // Measurement matrix (we only measure position, not velocity)
        H_ = Eigen::MatrixXd::Zero(3, 6);
        H_(0, 0) = 1.0;  // x
        H_(1, 1) = 1.0;  // y
        H_(2, 2) = 1.0;  // z

        initialized_ = false;
    }

    /**
     * @brief Initialize the filter with first measurement
     */
    void initialize(const geometry_msgs::msg::Point& position) {
        state_(0) = position.x;
        state_(1) = position.y;
        state_(2) = position.z;
        state_(3) = 0.0;  // vx
        state_(4) = 0.0;  // vy
        state_(5) = 0.0;  // vz
        initialized_ = true;
    }

    /**
     * @brief Predict step - propagate state forward in time
     * @param dt Time step in seconds
     */
    void predict(double dt) {
        if (!initialized_) return;

        // State transition matrix (constant velocity model)
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = dt;  // x = x + vx * dt
        F(1, 4) = dt;  // y = y + vy * dt
        F(2, 5) = dt;  // z = z + vz * dt

        // Predict state
        state_ = F * state_;

        // Predict covariance
        P_ = F * P_ * F.transpose() + Q_;
    }

    /**
     * @brief Update step - correct prediction with measurement
     * @param position Measured position
     */
    void update(const geometry_msgs::msg::Point& position) {
        if (!initialized_) {
            initialize(position);
            return;
        }

        // Measurement vector
        Eigen::VectorXd z(3);
        z(0) = position.x;
        z(1) = position.y;
        z(2) = position.z;

        // Innovation (measurement residual)
        Eigen::VectorXd y = z - H_ * state_;

        // Innovation covariance
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;

        // Kalman gain
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        // Update state estimate
        state_ = state_ + K * y;

        // Update covariance estimate
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P_ = (I - K * H_) * P_;
    }

    /**
     * @brief Get estimated position
     */
    geometry_msgs::msg::Point getPosition() const {
        geometry_msgs::msg::Point pos;
        pos.x = state_(0);
        pos.y = state_(1);
        pos.z = state_(2);
        return pos;
    }

    /**
     * @brief Get estimated velocity
     */
    geometry_msgs::msg::Vector3 getVelocity() const {
        geometry_msgs::msg::Vector3 vel;
        vel.x = state_(3);
        vel.y = state_(4);
        vel.z = state_(5);
        return vel;
    }

    /**
     * @brief Get speed magnitude
     */
    double getSpeed() const {
        return std::sqrt(state_(3) * state_(3) +
                        state_(4) * state_(4) +
                        state_(5) * state_(5));
    }

    /**
     * @brief Check if filter is initialized
     */
    bool isInitialized() const {
        return initialized_;
    }

    /**
     * @brief Set process noise for position
     */
    void setProcessNoisePosition(double noise) {
        Q_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * noise;
    }

    /**
     * @brief Set process noise for velocity
     */
    void setProcessNoiseVelocity(double noise) {
        Q_.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * noise;
    }

    /**
     * @brief Set measurement noise
     */
    void setMeasurementNoise(double noise) {
        R_ = Eigen::MatrixXd::Identity(3, 3) * noise;
    }

private:
    Eigen::VectorXd state_;      // State vector [x, y, z, vx, vy, vz]
    Eigen::MatrixXd P_;          // State covariance
    Eigen::MatrixXd Q_;          // Process noise covariance
    Eigen::MatrixXd R_;          // Measurement noise covariance
    Eigen::MatrixXd H_;          // Measurement matrix
    bool initialized_;
};

}  // namespace person_tracker

#endif  // PERSON_TRACKER__KALMAN_FILTER_HPP_
