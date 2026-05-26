//
// Created by Or Salmon on 20/06/18.
//

#include "ErrorStateCovariance.h"

namespace EKF_INS {
ErrorStateCovariance::ErrorStateCovariance(std::shared_ptr<EKF_INS::ErrorState> error_state_ptr) : P_(15, 15), Q_(15, 15) {
  error_state_ptr_ = error_state_ptr;
  // Initialize P with reasonable default uncertainties
  // Position: (5m)^2 for horizontal, (10m)^2 for altitude
  // Velocity: (0.5 m/s)^2 for each axis
  // Attitude: (3 deg)^2 for roll/pitch, (10 deg)^2 for yaw (for land vehicles)
  // Accel bias: (0.1 m/s^2)^2
  // Gyro bias: (0.01 rad/s)^2 = (0.57 deg/s)^2
  P_.diagonal() << std::pow(5.0, 2),    std::pow(5.0, 2),    std::pow(10.0, 2),
                  std::pow(0.5, 2),    std::pow(0.5, 2),    std::pow(0.5, 2),
                  std::pow(3.0*M_PI/180.0, 2), std::pow(3.0*M_PI/180.0, 2), std::pow(10.0*M_PI/180.0, 2),
                  std::pow(0.1, 2),    std::pow(0.1, 2),    std::pow(0.1, 2),
                  std::pow(0.01, 2),   std::pow(0.01, 2),   std::pow(0.01, 2);
  Q_.setZero();
}

ErrorStateCovariance::ErrorStateCovariance(std::shared_ptr<EKF_INS::ErrorState> error_state_ptr, const Eigen::MatrixXd &Q) : ErrorStateCovariance(
    error_state_ptr) {
  setQMatrix(Q);
}

void ErrorStateCovariance::setQMatrix(const Eigen::MatrixXd &Q) { Q_ = Q; }

void ErrorStateCovariance::updateCovarianceMatrix(double dt) {
  Eigen::MatrixXd phi = error_state_ptr_->getTransitionMatrix(dt);
  // Process noise discretization (first-order Euler approximation):
  //   Q_d ≈ Q_c * dt
  // where Q_c is the CONTINUOUS-TIME process noise spectral density (15x15 diagonal
  // set via setQMatrix / setCovarianceDt). The full G*Q_c*G^T projection through the
  // noise input matrix G is deferred to future work — for now G = I (noise applied
  // directly to error states).
  //
  // The covariance dt can be independently set via setCovarianceDt(). When set, it
  // overrides the IMU propagation dt for covariance prediction only.
  P_ = phi * P_ * phi.transpose() + Q_ * dt;
}

Eigen::MatrixXd ErrorStateCovariance::getErrorStateCovariance() { return P_; }

void ErrorStateCovariance::setPMatrix(const Eigen::MatrixXd &P) {
  P_ = P;
}
} // namespace EKF_INS
