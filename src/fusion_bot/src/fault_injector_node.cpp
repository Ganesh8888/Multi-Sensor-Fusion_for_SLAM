#include <random>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class FaultInjectorNode : public rclcpp::Node
{
public:
  FaultInjectorNode()
  : Node("fault_injector_node")
  {
    // Declare dynamic parameters
    this->declare_parameter<bool>("fault_wheel_slip", false);
    this->declare_parameter<double>("noise_level", 0.05);

    // Create subscriber and publisher
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/skid_steer_controller/odom", 10,
      std::bind(&FaultInjectorNode::odom_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_noisy", 10);

    RCLCPP_INFO(this->get_logger(), "Fault Injector Node initialized.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Retrieve dynamic parameters
    bool fault_active = this->get_parameter("fault_wheel_slip").as_bool();
    double noise_std = this->get_parameter("noise_level").as_double();

    auto noisy_msg = *msg;

    if (fault_active) {
      // Setup random number generator
      std::random_device rd;
      std::default_random_engine generator(rd());
      std::normal_distribution<double> distribution(0.0, noise_std);

      // Inject noise into positions (x, y)
      noisy_msg.pose.pose.position.x += distribution(generator);
      noisy_msg.pose.pose.position.y += distribution(generator);

      // Inject noise into linear velocity (x)
      noisy_msg.twist.twist.linear.x += distribution(generator);

      // Artificially increase covariance to indicate high uncertainty/noise
      // ROS 2 Odometry covariance is a 36-element array (6x6 matrix)
      // Pose covariance index: X=0, Y=7, Yaw=35
      noisy_msg.pose.covariance[0] = 5.0;  // High uncertainty in X
      noisy_msg.pose.covariance[7] = 5.0;  // High uncertainty in Y
      noisy_msg.pose.covariance[35] = 1.0; // High uncertainty in Yaw

      // Twist covariance index: Linear X=0, Angular Z=35
      noisy_msg.twist.covariance[0] = 2.0;
      noisy_msg.twist.covariance[35] = 1.0;

      RCLCPP_DEBUG(this->get_logger(), "Injected noise (std_dev=%f) into odometry.", noise_std);
    }

    odom_pub_->publish(noisy_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FaultInjectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
