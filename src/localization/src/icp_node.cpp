/*
 * icp_node.cpp — LiDAR scan-matching correction for EKF localization
 *
 * Subscriptions:
 *   /scan  (sensor_msgs/LaserScan)      — 2-D LiDAR
 *   /odom  (nav_msgs/Odometry)          — continuous pose from ekf_localization
 *
 * Publications:
 *   /pose_measurement  (geometry_msgs/PoseWithCovarianceStamped)
 *       → fed back into ekf_localization as a pose correction
 *
 * How it works
 * ─────────────
 * 1. Each scan is converted to a 2-D point cloud in the world frame
 *    using the latest /odom estimate.
 * 2. The cloud is matched against a stored "keyframe" cloud (also in world
 *    frame) using 2-D ICP.
 * 3. The ICP correction (dx, dy, dθ) is composed with the raw odometry pose
 *    to produce a corrected pose, published to /pose_measurement.
 * 4. The EKF fuses this measurement with its wheel-odometry prediction,
 *    keeping the /odom estimate accurate over time.
 * 5. The keyframe is updated whenever the robot has moved more than
 *    keyframe_dist metres or keyframe_angle radians since the last key frame,
 *    ensuring scans always have good overlap with the reference.
 *
 * Pose-correction maths
 * ─────────────────────
 * If T_odom = (ox, oy, θ_o) and ICP returns (dx, dy, dθ), the corrected
 * pose is the SE(2) composition  T_icp ∘ T_odom :
 *
 *   x_c     = cos(dθ)·ox − sin(dθ)·oy + dx
 *   y_c     = sin(dθ)·ox + cos(dθ)·oy + dy
 *   θ_c     = wrap(θ_o + dθ)
 */

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "puzzlebot_localization_cpp/icp2d.hpp"

using std::placeholders::_1;
using namespace puzzlebot_localization_cpp;

// ── Helpers ──────────────────────────────────────────────────────────────────

static double wrap(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

// ── Node ─────────────────────────────────────────────────────────────────────

class IcpNode : public rclcpp::Node
{
public:
    IcpNode() : Node("icp_node")
    {
        // ── Declare and cache parameters ──────────────────────────────
        declare_parameter("range_min",        0.12);
        declare_parameter("range_max",        10.0);
        declare_parameter("icp_max_iter",     20);
        declare_parameter("icp_tolerance",    1e-4);
        declare_parameter("icp_reject_dist",  0.3);   // m — reject far pairs
        declare_parameter("icp_min_points",   30);    // minimum inliers
        declare_parameter("icp_max_fitness",  0.15);  // m — reject bad fits
        declare_parameter("keyframe_dist",    0.3);   // m — update threshold
        declare_parameter("keyframe_angle",   0.2);   // rad (~11°)
        declare_parameter("cov_xy",           0.02);  // m² — published covariance
        declare_parameter("cov_theta",        0.01);  // rad²
        declare_parameter("base_frame",       std::string("base_link"));

        range_min_       = get_parameter("range_min").as_double();
        range_max_       = get_parameter("range_max").as_double();
        icp_max_iter_    = get_parameter("icp_max_iter").as_int();
        icp_tolerance_   = get_parameter("icp_tolerance").as_double();
        icp_reject_dist_ = get_parameter("icp_reject_dist").as_double();
        icp_min_points_  = get_parameter("icp_min_points").as_int();
        icp_max_fitness_ = get_parameter("icp_max_fitness").as_double();
        keyframe_dist_   = get_parameter("keyframe_dist").as_double();
        keyframe_angle_  = get_parameter("keyframe_angle").as_double();
        cov_xy_          = get_parameter("cov_xy").as_double();
        cov_theta_       = get_parameter("cov_theta").as_double();
        base_frame_      = get_parameter("base_frame").as_string();

        // ── TF: lazy laser_frame → base_frame lookup ──────────────────
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // ── Subscriptions ─────────────────────────────────────────────
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&IcpNode::odom_callback, this, _1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&IcpNode::scan_callback, this, _1));

        // ── Publisher ─────────────────────────────────────────────────
        meas_pub_ = create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_measurement", 10);

        RCLCPP_INFO(get_logger(),
            "ICP node ready — /scan + /odom → /pose_measurement  "
            "(keyframe: %.2f m / %.1f°)",
            keyframe_dist_, keyframe_angle_ * 180.0 / M_PI);
    }

private:
    // ── Callbacks ─────────────────────────────────────────────────────────────

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ox_     = msg->pose.pose.position.x;
        oy_     = msg->pose.pose.position.y;
        otheta_ = yaw_from_quat(msg->pose.pose.orientation);
        odom_ready_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_ready_) return;

        if (!resolve_laser_offset(msg->header.frame_id)) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for TF " << base_frame_ << "→" << msg->header.frame_id);
            return;
        }

        scan_count_++;

        // ── Build world-frame point cloud (uses current — possibly drifted — odom)
        const Eigen::MatrixXd cloud_robot = scan_to_points(
            msg->ranges, msg->angle_min, msg->angle_increment,
            range_min_, range_max_,
            laser_x_, laser_y_, laser_yaw_);

        if (cloud_robot.rows() < icp_min_points_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Scan has only %ld valid points (< %d) — skipping",
                cloud_robot.rows(), icp_min_points_);
            return;
        }

        Eigen::MatrixXd cloud_world =
            transform_points(cloud_robot, ox_, oy_, otheta_);

        // ── Initialise keyframe on first scan ─────────────────────────
        if (!has_keyframe_) {
            set_keyframe(cloud_world, ox_, oy_, otheta_);
            RCLCPP_INFO(get_logger(),
                "First keyframe set @ (%.2f, %.2f, %.1f°) — %ld points",
                ox_, oy_, otheta_ * 180.0 / M_PI, cloud_world.rows());
            return;
        }

        // ── ICP: align current cloud → keyframe cloud ─────────────────
        const IcpResult r = icp(
            cloud_world, keyframe_cloud_,
            icp_max_iter_, icp_tolerance_,
            icp_reject_dist_, icp_min_points_);

        // Pose used for the keyframe update at the end of this callback.
        // Defaults to the (drifted) current odom; overridden if ICP succeeds.
        double anchor_x = ox_, anchor_y = oy_, anchor_theta = otheta_;
        Eigen::MatrixXd anchor_cloud = cloud_world;

        if (r.converged && r.fitness < icp_max_fitness_) {
            publish_correction(r, msg->header.stamp);
            corrections_published_++;

            // BUGFIX: bring our local pose state in line with the correction we
            // just sent.  Otherwise the next set_keyframe() anchors to the
            // *drifted* pose, so the next ICP iteration produces an opposite
            // correction that bounces the EKF back to where it was.
            const double c   = std::cos(r.dtheta);
            const double s   = std::sin(r.dtheta);
            anchor_x         = c * ox_ - s * oy_ + r.dx;
            anchor_y         = s * ox_ + c * oy_ + r.dy;
            anchor_theta     = wrap(otheta_ + r.dtheta);

            // Apply the same SE(2) transform to cloud_world so it lives in
            // the corrected world frame (which is what subsequent ICPs match against).
            anchor_cloud = transform_points(cloud_world, r.dx, r.dy, r.dtheta);

            // Mirror the EKF update locally so this scan's keyframe-update test
            // and any concurrent scans use the corrected state until the next
            // /odom message arrives.
            ox_ = anchor_x;
            oy_ = anchor_y;
            otheta_ = anchor_theta;

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[ICP] OK  fit=%.3f  Δ=(%+.3f, %+.3f, %+.2f°)  pose→(%.2f, %.2f, %.1f°)  "
                "[%d/%d corrections]",
                r.fitness, r.dx, r.dy, r.dtheta * 180.0 / M_PI,
                anchor_x, anchor_y, anchor_theta * 180.0 / M_PI,
                corrections_published_, scan_count_);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[ICP] rejected — converged=%d  fitness=%.3f  src_pts=%ld  kf_pts=%ld",
                static_cast<int>(r.converged), r.fitness,
                cloud_world.rows(), keyframe_cloud_.rows());
        }

        // ── Update keyframe when robot has moved enough ────────────────
        const double dist  = std::hypot(anchor_x - kx_, anchor_y - ky_);
        const double angle = std::abs(wrap(anchor_theta - ktheta_));
        if (dist > keyframe_dist_ || angle > keyframe_angle_) {
            set_keyframe(anchor_cloud, anchor_x, anchor_y, anchor_theta);
            RCLCPP_INFO(get_logger(),
                "Keyframe updated @ (%.2f, %.2f, %.1f°) — %ld pts (Δdist=%.2f m, Δang=%.1f°)",
                anchor_x, anchor_y, anchor_theta * 180.0 / M_PI,
                anchor_cloud.rows(), dist, angle * 180.0 / M_PI);
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    bool resolve_laser_offset(const std::string & laser_frame)
    {
        if (laser_offset_ready_) return true;
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                base_frame_, laser_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException &) {
            return false;
        }
        laser_x_ = tf.transform.translation.x;
        laser_y_ = tf.transform.translation.y;
        const auto & q = tf.transform.rotation;
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        laser_yaw_ = std::atan2(siny, cosy);
        laser_offset_ready_ = true;
        RCLCPP_INFO(get_logger(),
            "Laser→%s offset cached: x=%.3f y=%.3f yaw=%.1f°",
            base_frame_.c_str(), laser_x_, laser_y_,
            laser_yaw_ * 180.0 / M_PI);
        return true;
    }

    void set_keyframe(const Eigen::MatrixXd & cloud,
                      double anchor_x, double anchor_y, double anchor_theta)
    {
        keyframe_cloud_ = cloud;
        kx_      = anchor_x;
        ky_      = anchor_y;
        ktheta_  = anchor_theta;
        has_keyframe_ = true;
    }

    void publish_correction(const IcpResult & r, const rclcpp::Time & stamp)
    {
        // Corrected pose = T_icp ∘ T_odom  (full SE(2) composition)
        const double c       = std::cos(r.dtheta);
        const double s       = std::sin(r.dtheta);
        const double x_c     = c * ox_ - s * oy_ + r.dx;
        const double y_c     = s * ox_ + c * oy_ + r.dy;
        const double theta_c = wrap(otheta_ + r.dtheta);

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = x_c;
        msg.pose.pose.position.y = y_c;
        msg.pose.pose.position.z = 0.0;

        const double half = theta_c / 2.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = std::sin(half);
        msg.pose.pose.orientation.w = std::cos(half);

        // 6×6 covariance, row-major, order: x y z roll pitch yaw
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0]  = cov_xy_;     // x–x
        msg.pose.covariance[7]  = cov_xy_;     // y–y
        msg.pose.covariance[35] = cov_theta_;  // yaw–yaw

        meas_pub_->publish(msg);
    }

    // ── Parameters (cached) ───────────────────────────────────────────────────
    double range_min_, range_max_;
    int    icp_max_iter_, icp_min_points_;
    double icp_tolerance_, icp_reject_dist_, icp_max_fitness_;
    double keyframe_dist_, keyframe_angle_;
    double cov_xy_, cov_theta_;

    // ── State ─────────────────────────────────────────────────────────────────
    double ox_{0.0}, oy_{0.0}, otheta_{0.0};
    bool   odom_ready_{false};

    Eigen::MatrixXd keyframe_cloud_;
    double kx_{0.0}, ky_{0.0}, ktheta_{0.0};
    bool   has_keyframe_{false};

    // Cached laser_frame → base_frame static offset
    std::string base_frame_{"base_link"};
    double laser_x_{0.0}, laser_y_{0.0}, laser_yaw_{0.0};
    bool   laser_offset_ready_{false};

    // Diagnostics
    int scan_count_{0};
    int corrections_published_{0};

    // ── ROS interfaces ────────────────────────────────────────────────────────
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr         scan_sub_;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr        meas_pub_;
};

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcpNode>());
    rclcpp::shutdown();
    return 0;
}
