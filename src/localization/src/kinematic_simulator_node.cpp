#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

class KinematicSimulatorNode : public rclcpp::Node
{
public:
  KinematicSimulatorNode()
  : Node("kinematic_simulator")
  {
    declare_parameter("wheel_radius",    0.05);
    declare_parameter("wheel_separation", 0.19);

    r_ = get_parameter("wheel_radius").as_double();
    L_ = get_parameter("wheel_separation").as_double();

    wr_pub_ = create_publisher<std_msgs::msg::Float32>("/wr", 10);
    wl_pub_ = create_publisher<std_msgs::msg::Float32>("/wl", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&KinematicSimulatorNode::cmd_vel_callback, this, _1));

    RCLCPP_INFO(get_logger(),
      "Kinematic Simulator started — r=%.3f m, L=%.3f m", r_, L_);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v     = msg->linear.x;
    double omega = msg->angular.z;

    std_msgs::msg::Float32 wr_msg, wl_msg;
    wr_msg.data = static_cast<float>((2.0 * v + omega * L_) / (2.0 * r_));
    wl_msg.data = static_cast<float>((2.0 * v - omega * L_) / (2.0 * r_));

    wr_pub_->publish(wr_msg);
    wl_pub_->publish(wl_msg);
  }

  double r_{0.05};
  double L_{0.19};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wl_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
