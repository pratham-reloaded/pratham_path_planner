#include <cstdio>
#include <iostream>

#include "pratham_path_planner/test.hpp"

#include "ros_api/occupancy_grid.hpp"

#include "path_planning/d_star_lite.hpp"
#include "../include/path_planning_lib/src/d_star_lite.cpp"

#include "utils/utils.hpp"
#include "../include/path_planning_lib/lib/utils/src/utils.cpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  constexpr int n = 201;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);
  PrintGrid(grid);
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
