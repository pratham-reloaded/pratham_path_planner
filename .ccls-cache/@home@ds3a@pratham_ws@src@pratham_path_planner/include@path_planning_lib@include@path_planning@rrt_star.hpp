/**
 * @file rrt_star.hpp
 * @author vss2sn
 * @brief Contains the RRTStar class
 */

#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <limits>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for objects that plan using the RRT* algorithm
 */
class RRTStar : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit RRTStar(std::vector<std::vector<int>> grid)
      : Planner(std::move(grid)) {}

  void SetParams(const int threshold = 2, const int max_iter_x_factor = 20);

  /**
   * @brief RRT* algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Grid>> Plan(const Grid& start,
                                           const Grid& goal) override;

 private:
  /**
   * @brief Find the nearest Grid that has been seen by the algorithm. This does
   * not consider cost to reach the node.
   * @param new_node Grid to which the nearest node must be found
   * @return nearest node
   */
  std::tuple<bool, Grid> FindNearestPoint(Grid& new_node);

  /**
   * @brief Check if there is any obstacle between the 2 nodes. As this planner
   * is for grid maps, the obstacles are square.
   * @param n_1 Grid 1
   * @param n_2 Grid 2
   * @return bool value of whether obstacle exists between nodes
   */
  bool IsAnyObstacleInPath(const Grid& n_1, const Grid& n_2) const;

  /**
   * @brief Generates a random node
   * @return Generated node
   */
  Grid GenerateRandomGrid() const;

  /**
   * @brief Rewire the tree
   * @param new_node Grid to which other nodes will be connected if their cost
   * decreases
   * @return void
   */
  void Rewire(const Grid& new_node);

  /**
   * @brief Check if goal is reachable from current node
   * @param new_node Current node
   * @return bool value of whether goal is reachable from current node
   */
  bool CheckGoalVisible(const Grid& new_node);

  /**
   * @brief Create the obstacle list from the input grid
   * @return void
   */
  void CreateObstacleList();

  std::vector<Grid> CreatePath();

 private:
  Grid start_, goal_;
  std::unordered_set<Grid, GridIdAsHash, compare_coordinates>
      point_list_;  // TODO: set up in cstor
  std::unordered_map<Grid, std::vector<Grid>> near_nodes_;
  std::vector<Grid> obstacle_list_;
  double threshold_ = 1.5;       // TODO: set up in cstor
  int max_iter_x_factor_ = 500;  // TODO: set up in cstor
};

#endif  // RRT_STAR_H
