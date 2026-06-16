#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToStampedNode : public rclcpp::Node
{
public:
  TwistToStampedNode()
  : Node("twist_to_stamped_node")
  {
    // Subscribe to standard unstamped /cmd_vel (from keyboard or Nav2)
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TwistToStampedNode::callback, this, std::placeholders::_1));

    // Publish to stamped controller input
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/skid_steer_controller/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Twist to Stamped Bridge Node initialized.");
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped_msg;
    stamped_msg.header.stamp = this->now();
    stamped_msg.header.frame_id = "base_footprint";
    stamped_msg.twist = *msg;
    pub_->publish(stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToStampedNode>());
  rclcpp::shutdown();
  return 0;
}
