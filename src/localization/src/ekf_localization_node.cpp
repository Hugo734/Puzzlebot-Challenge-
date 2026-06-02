#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "puzzlebot_localization_cpp/ekf.hpp"

using std::placeholders::_1;

class EKFLocalizationNode : public rclcpp::Node
{
public:
  EKFLocalizationNode()
  : Node("ekf_localization")
  {
    declare_parameter("wheel_radius",    0.05);
    declare_parameter("wheel_separation", 0.19);
    declare_parameter("update_rate",     20.0);
    declare_parameter("q_x",            0.01);
    declare_parameter("q_y",            0.01);
    declare_parameter("q_theta",        0.005);
    declare_parameter("r_x",            0.05);
    declare_parameter("r_y",            0.05);
    declare_parameter("r_theta",        0.02);

    r_   = get_parameter("wheel_radius").as_double();
    L_   = get_parameter("wheel_separation").as_double();
    dt_  = 1.0 / get_parameter("update_rate").as_double();

    ekf_.setProcessNoise(
      get_parameter("q_x").as_double(),
      get_parameter("q_y").as_double(),
      get_parameter("q_theta").as_double());

    ekf_.setMeasurementNoise(
      get_parameter("r_x").as_double(),
      get_parameter("r_y").as_double(),
      get_parameter("r_theta").as_double());

    wr_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wr", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { wr_ = msg->data; });

    wl_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wl", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { wl_ = msg->data; });

    meas_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose_measurement", 10,
      std::bind(&EKFLocalizationNode::measurement_callback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&EKFLocalizationNode::timer_callback, this));

    RCLCPP_INFO(get_logger(),
      "EKF Localization started — r=%.3f m, L=%.3f m, dt=%.3f s", r_, L_, dt_);
  }

private:
  void timer_callback()
  {
    ekf_.predict(wr_, wl_, r_, L_, dt_);

    const auto & x = ekf_.state();
    const auto & P = ekf_.covariance();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x(2));

    auto now = get_clock()->now();

    // --- Odometry message ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";

    odom.pose.pose.position.x    = x(0);
    odom.pose.pose.position.y    = x(1);
    odom.pose.pose.position.z    = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 6x6 pose covariance (row-major, order: x y z roll pitch yaw)
    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0]  = P(0, 0);   // x-x
    odom.pose.covariance[1]  = P(0, 1);   // x-y
    odom.pose.covariance[5]  = P(0, 2);   // x-yaw
    odom.pose.covariance[6]  = P(1, 0);   // y-x
    odom.pose.covariance[7]  = P(1, 1);   // y-y
    odom.pose.covariance[11] = P(1, 2);   // y-yaw
    odom.pose.covariance[30] = P(2, 0);   // yaw-x
    odom.pose.covariance[31] = P(2, 1);   // yaw-y
    odom.pose.covariance[35] = P(2, 2);   // yaw-yaw

    double v     = r_ * (wr_ + wl_) / 2.0;
    double omega = r_ * (wr_ - wl_) / L_;
    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = omega;

    odom_pub_->publish(odom);

    // --- TF broadcast ---
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_footprint";
    tf.transform.translation.x = x(0);
    tf.transform.translation.y = x(1);
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  void measurement_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto & ori = msg->pose.pose.orientation;
    double siny = 2.0 * (ori.w * ori.z + ori.x * ori.y);
    double cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z);
    double yaw  = std::atan2(siny, cosy);

    ekf_.update(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yaw);

    RCLCPP_DEBUG(get_logger(), "EKF correction applied from /pose_measurement");
  }

  puzzlebot_localization_cpp::EKF ekf_;

  double r_{0.05};
  double L_{0.19};
  double dt_{0.05};
  double wr_{0.0};
  double wl_{0.0};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wr_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr meas_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
