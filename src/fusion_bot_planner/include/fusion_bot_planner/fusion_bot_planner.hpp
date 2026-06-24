#ifndef FUSION_BOT_PLANNER__FUSION_BOT_PLANNER_HPP_
#define FUSION_BOT_PLANNER__FUSION_BOT_PLANNER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <functional>
#include <queue>
#include <cmath>
#include <algorithm>

namespace fusion_bot_planner
{

struct AStarNode
{
  unsigned int x, y; // Grid coordinates on the map
  float g_cost;      // Distance traveled from the start
  float f_cost;      // g_cost + estimated distance to the goal
  
  // A pointer to remember which pixel we came from (so we can draw the path backwards at the end)
  std::shared_ptr<AStarNode> parent;

  // We want the node with the LOWEST f_cost to be explored first.
  bool operator>(const AStarNode& other) const {
    return f_cost > other.f_cost;
  }
};

// We inherit from nav2_core::GlobalPlanner
class FusionBotPlanner : public nav2_core::GlobalPlanner
{
public:
  FusionBotPlanner() = default;
  ~FusionBotPlanner() = default;

  // 1. Configure is called when the Lifecycle Node starts. We get parameters here.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // 2. Cleanup is called when the node shuts down to free memory.
  void cleanup() override;

  // 3. Activate is called when the robot is ready to drive.
  void activate() override;

  // 4. Deactivate is called to pause the planner.
  void deactivate() override;

  // 5. THIS IS THE BRAIN! Nav2 calls this function every time it needs a path.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // We store these internally to use them in the createPlan function later
  nav2_costmap_2d::Costmap2D * costmap_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("FusionBotPlanner")};
  std::string global_frame_, name_;
};

}  // namespace fusion_bot_planner

#endif  // FUSION_BOT_PLANNER__FUSION_BOT_PLANNER_HPP_
