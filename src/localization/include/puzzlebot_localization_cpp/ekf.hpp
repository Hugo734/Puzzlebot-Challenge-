#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace puzzlebot_localization_cpp
{

// Extended Kalman Filter for a differential-drive robot.
// State: x = [x, y, theta]^T  (pose in odom frame)
class EKF
{
public:
  EKF()
  {
    x_ = Eigen::Vector3d::Zero();
    P_ = Eigen::Matrix3d::Identity() * 1e-4;
    Q_ = Eigen::Matrix3d::Identity() * 0.01;
    R_ = Eigen::Matrix3d::Identity() * 0.05;
  }

  void setProcessNoise(double qx, double qy, double qtheta)
  {
    Q_ = Eigen::Matrix3d::Zero();
    Q_(0, 0) = qx;
    Q_(1, 1) = qy;
    Q_(2, 2) = qtheta;
  }

  void setMeasurementNoise(double rx, double ry, double rtheta)
  {
    R_ = Eigen::Matrix3d::Zero();
    R_(0, 0) = rx;
    R_(1, 1) = ry;
    R_(2, 2) = rtheta;
  }

  // Predict using wheel angular velocities (rad/s) and timestep dt (s).
  void predict(double wr, double wl, double r, double L, double dt)
  {
    double v     = r * (wr + wl) / 2.0;
    double omega = r * (wr - wl) / L;
    double theta = x_(2);

    x_(0) += v * std::cos(theta) * dt;
    x_(1) += v * std::sin(theta) * dt;
    x_(2) = wrapAngle(x_(2) + omega * dt);

    // Jacobian of motion model wrt state
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -v * std::sin(theta) * dt;
    F(1, 2) =  v * std::cos(theta) * dt;

    P_ = F * P_ * F.transpose() + Q_;
  }

  // Update using a full pose measurement z = [x_m, y_m, theta_m].
  // H = I, so the observation is the state directly.
  void update(double x_m, double y_m, double theta_m)
  {
    Eigen::Vector3d z(x_m, y_m, theta_m);

    Eigen::Vector3d innov = z - x_;
    innov(2) = wrapAngle(innov(2));

    // S = P + R  (since H = I)
    Eigen::Matrix3d S = P_ + R_;
    Eigen::Matrix3d K = P_ * S.inverse();

    x_ = x_ + K * innov;
    x_(2) = wrapAngle(x_(2));

    P_ = (Eigen::Matrix3d::Identity() - K) * P_;
  }

  const Eigen::Vector3d & state()      const { return x_; }
  const Eigen::Matrix3d & covariance() const { return P_; }

  void setState(double x, double y, double theta)
  {
    x_ << x, y, theta;
  }

private:
  static double wrapAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  Eigen::Vector3d x_;   // [x, y, theta]
  Eigen::Matrix3d P_;   // pose covariance
  Eigen::Matrix3d Q_;   // process noise covariance
  Eigen::Matrix3d R_;   // measurement noise covariance
};

}  // namespace puzzlebot_localization_cpp
