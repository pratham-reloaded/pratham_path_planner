#include <cstdio>
#include <iostream>

#include "pratham_path_planner/test.hpp"

#include "ros_api/occupancy_grid.hpp"

#include "path_planning/d_star_lite.hpp"
#include "../include/path_planning_lib/src/d_star_lite.cpp"

#include "utils/utils.hpp"
#include "../include/path_planning_lib/lib/utils/src/utils.cpp"

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"

using namespace::std::chrono_literals;

#define GRID_SIZE 320

typedef nav_msgs::msg::OccupancyGrid OccupancyGridMsg;
typedef geometry_msgs::msg::Pose PoseMsg;

/*
 * TODO
 * - start a ros2 node
 * - subscribe to /tf, /costmap, and /goal
 * - create a publisher to the /path/raw topic
 * */

class PathPlanner : public rclcpp::Mode
{
  public:
    PathPlanner()
    : Node("path_planner")
    {
      
    }
}

int main(int argc, char **argv)
{
  (void) argc;
  (void) argv;
  constexpr int n = GRID_SIZE;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);
  // PrintGrid(grid);
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Node start(distr(eng), distr(eng), 0, 0, 0, 0);
  Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

 DStarLite dstar_lite(grid);
  {
    const auto [path_found, path_vector] = dstar_lite.Plan(start, goal);
    for (int i=0; i<path_vector.size(); i++)
      printf("%d %d\n", path_vector[i].x_, path_vector[i].y_);
  }
  return 0;
}
