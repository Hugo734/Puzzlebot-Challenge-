#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

// Bridges real Jetson encoder topics to the standard /wr, /wl interface.
//   /VelocityEncl  -> /wl   (left wheel,  rad/s)
//   /VelocityEncnR -> /wr   (right wheel, rad/s)
class VelocityBridgeNode : public rclcpp::Node
{
public:
  VelocityBridgeNode()
  : Node("velocity_bridge")
  {
    wr_pub_ = create_publisher<std_msgs::msg::Float32>("/wr", 10);
    wl_pub_ = create_publisher<std_msgs::msg::Float32>("/wl", 10);

    left_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/VelocityEncl", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { wl_pub_->publish(*msg); });

    right_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/VelocityEncnR", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { wr_pub_->publish(*msg); });

    RCLCPP_INFO(get_logger(),
      "Velocity Bridge started — /VelocityEncl -> /wl | /VelocityEncnR -> /wr");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wl_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
