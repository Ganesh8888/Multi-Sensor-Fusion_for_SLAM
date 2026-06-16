#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

class DiagnosticsNode : public rclcpp::Node
{
public:
  DiagnosticsNode()
  : Node("diagnostics_node")
  {
    // Initialize last message times to the current time (so they don't immediately error on startup)
    auto now = this->now();
    last_imu_time_ = now;
    last_lidar_time_ = now;
    last_camera_time_ = now;
    last_odom_time_ = now;

    // Create subscriptions
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&DiagnosticsNode::imu_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DiagnosticsNode::lidar_callback, this, std::placeholders::_1));

    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10, std::bind(&DiagnosticsNode::camera_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/skid_steer_controller/odom", 10, std::bind(&DiagnosticsNode::odom_callback, this, std::placeholders::_1));

    // Create publisher
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // Timer running at 1.0 Hz (1000ms) to check health status
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&DiagnosticsNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Diagnostics & Monitor Node initialized.");
  }

private:
  // Subscription Callbacks: Just update the timestamp to the current time when data arrives
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    (void)msg; // Suppress compiler warning for unused variable
    last_imu_time_ = this->now();
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    (void)msg;
    last_lidar_time_ = this->now();
  }

  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    (void)msg;
    last_camera_time_ = this->now();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    (void)msg;
    last_odom_time_ = this->now();
  }

  // Timer Callback (Runs every 1 second)
  void timer_callback()
  {
    auto now = this->now();
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now;

    // Define timeouts (in seconds)
    const double imu_timeout = 0.15; // IMU is 100Hz (0.01s), warning if no messages for 0.15s
    const double lidar_timeout = 0.5; // LiDAR is 10Hz (0.1s), warning if no messages for 0.5s
    const double camera_timeout = 0.5; // Camera is 30Hz, warning if no messages for 0.5s
    const double odom_timeout = 0.5;  // Odom is 30Hz, warning if no messages for 0.5s

    // Check each sensor and build status messages
    diag_array.status.push_back(check_sensor("IMU Sensor", now, last_imu_time_, imu_timeout, "imu_link"));
    diag_array.status.push_back(check_sensor("LiDAR Sensor", now, last_lidar_time_, lidar_timeout, "laser_frame"));
    diag_array.status.push_back(check_sensor("Depth Camera Sensor", now, last_camera_time_, camera_timeout, "camera_link"));
    diag_array.status.push_back(check_sensor("Wheel Odometry", now, last_odom_time_, odom_timeout, "odom"));

    diag_pub_->publish(diag_array);
  }

  // Helper function to build a DiagnosticStatus message for a single sensor
  diagnostic_msgs::msg::DiagnosticStatus check_sensor(
    const std::string & sensor_name,
    const rclcpp::Time & now,
    const rclcpp::Time & last_received,
    double timeout_limit,
    const std::string & hardware_id)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = sensor_name;
    status.hardware_id = hardware_id;

    double elapsed = (now - last_received).seconds();

    if (elapsed > timeout_limit) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "TIMEOUT: No data received for " + std::to_string(elapsed) + " seconds.";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Active: Frequency is normal.";
    }

    // Add key-value pair for detailed frequency view in telemetry
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "Seconds since last message";
    kv.value = std::to_string(elapsed);
    status.values.push_back(kv);

    return status;
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Timestamps
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_lidar_time_;
  rclcpp::Time last_camera_time_;
  rclcpp::Time last_odom_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiagnosticsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
