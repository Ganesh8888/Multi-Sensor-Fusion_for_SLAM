// 1. Include the header file we just wrote
#include "fusion_bot_planner/fusion_bot_planner.hpp"

// 2. We use the namespace we defined in the header
namespace fusion_bot_planner
{

// 3. The "configure" function. This runs once when the robot turns on.
void FusionBotPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  tf_ = tf;
  
  // EXTRACT THE COSTMAP DATA HERE!
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = parent.lock();
  clock_ = node->get_clock();

  RCLCPP_INFO(logger_, "Configuring Karpaga's Custom Planner plugin...");
}

// 4. The "cleanup" function. Runs when shutting down.
void FusionBotPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin...");
}

// 5. The "activate" function. Runs when the robot is ready to move.
void FusionBotPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin...");
}

// 6. The "deactivate" function. Runs if we pause the robot.
void FusionBotPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin...");
}

// 7. THE BRAIN. This runs every time we tell the robot to move.
nav_msgs::msg::Path FusionBotPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  RCLCPP_INFO(logger_, "Calculating A* path...");

  // 1. Create the final Path object we will eventually return
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  // 2. Variables to hold our Map Pixels
  unsigned int start_x, start_y, goal_x, goal_y;

  // 3. Convert World (Meters) to Map (Pixels). 
  // If the user clicks a point outside the map, we safely exit.
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) 
  {
    RCLCPP_ERROR(logger_, "Start or goal is outside the map boundary!");
    return path; 
  }

  // 4. Create the C++ Priority Queue (The "Open Set"). 
  // This automatically sorts our AStarNodes so the cheapest one is always at the top!
  std::priority_queue<
    std::shared_ptr<AStarNode>, 
    std::vector<std::shared_ptr<AStarNode>>, 
    std::greater<std::shared_ptr<AStarNode>>> open_set;

  // 5. Create a 2D Array of booleans (The "Closed Set") to track pixels we already checked
  std::vector<std::vector<bool>> closed_set(
    costmap_->getSizeInCellsX(), 
    std::vector<bool>(costmap_->getSizeInCellsY(), false));

  // 6. Create the very first node (Where the robot is currently standing)
  auto start_node = std::make_shared<AStarNode>();
  start_node->x = start_x;
  start_node->y = start_y;
  start_node->g_cost = 0.0;
  start_node->f_cost = 0.0; 
  start_node->parent = nullptr; // It has no parent because it is the beginning

  // Put the start node into the queue to kick off the algorithm!
  open_set.push(start_node);
  
  // A helper function (Lambda) to calculate straight-line distance to the goal
  auto get_heuristic = [](unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
    return std::hypot(static_cast<float>(x1) - static_cast<float>(x2), 
                      static_cast<float>(y1) - static_cast<float>(y2));
  };

  // The 8 directions the robot can move (X, Y)
  std::vector<std::pair<int, int>> directions = {
    {0, 1}, {0, -1}, {1, 0}, {-1, 0},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  // KEEP SEARCHING AS LONG AS THE QUEUE IS NOT EMPTY
  while (!open_set.empty()) {
    
    // 1. Emergency Stop Check
    if (cancel_checker && cancel_checker()) {
      RCLCPP_WARN(logger_, "Path planning cancelled by system!");
      return path;
    }

    // 2. Pull the cheapest node out of the queue
    auto current_node = open_set.top();
    open_set.pop();

    // 3. Skip if we already checked this exact pixel
    if (closed_set[current_node->x][current_node->y]) {
      continue;
    }
    closed_set[current_node->x][current_node->y] = true;

    // 4. DID WE WIN? (Are we at the Goal?)
    if (current_node->x == goal_x && current_node->y == goal_y) {
      RCLCPP_INFO(logger_, "A* Path Found! Reconstructing route...");
      
      // Trace the parent pointers backwards to draw the path
      auto curr = current_node;
      while (curr != nullptr) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = clock_->now();
        
        // Convert Map Pixels back into Real World Meters!
        double wx, wy;
        costmap_->mapToWorld(curr->x, curr->y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        
        path.poses.push_back(pose);
        curr = curr->parent;
      }
      
      // Because we traced backward, the path is reversed. Flip it!
      std::reverse(path.poses.begin(), path.poses.end());
      return path;
    }

    // 5. EXPLORE THE 8 NEIGHBORS
    for (const auto& dir : directions) {
      unsigned int next_x = current_node->x + dir.first;
      unsigned int next_y = current_node->y + dir.second;

      // Make sure we don't accidentally check a pixel off the edge of the map
      if (next_x >= costmap_->getSizeInCellsX() || next_y >= costmap_->getSizeInCellsY()) {
        continue;
      }

      // Check if this pixel is a wall (254 = Lethal Obstacle, 253 = Inflated Obstacle, 255 = Unknown)
      unsigned char cost = costmap_->getCost(next_x, next_y);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || 
          cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || 
          cost == nav2_costmap_2d::NO_INFORMATION) {
        continue; // It's a wall. Skip it!
      }

      // It's safe! Create a new node.
      auto neighbor = std::make_shared<AStarNode>();
      neighbor->x = next_x;
      neighbor->y = next_y;
      neighbor->parent = current_node;
      
      // Math: Cost goes up by 1.414 for diagonals, and 1.0 for straight lines
      float step_cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0;
      neighbor->g_cost = current_node->g_cost + step_cost;
      neighbor->f_cost = neighbor->g_cost + get_heuristic(next_x, next_y, goal_x, goal_y);

      open_set.push(neighbor);
    }
  }

  // If the queue empties out and we never hit the goal, the path is completely blocked.
  RCLCPP_WARN(logger_, "Failed to find a path! The goal is surrounded by walls.");
  return path;
}

}  // namespace fusion_bot_planner

// 8. IMPORTANT: This macro "registers" our class with ROS 2 Pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fusion_bot_planner::FusionBotPlanner, nav2_core::GlobalPlanner)
