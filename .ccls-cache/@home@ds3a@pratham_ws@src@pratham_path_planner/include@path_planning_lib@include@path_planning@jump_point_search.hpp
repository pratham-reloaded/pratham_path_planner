/**
 * @file jump_point_search.hpp
 * @author vss2sn
 * @brief Contains the JumpPointSearch class
 */

#ifndef JUMP_POINT_SEARCH_H
#define JUMP_POINT_SEARCH_H

#include <queue>
#include <unordered_set>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for objects that plan using the jump point search algorithm
 */
class JumpPointSearch : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit JumpPointSearch(const std::vector<std::vector<int>>& grid)
      : Planner(grid){};

  /**
   * @brief Jump Point Search algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Grid>> Plan(const Grid& start,
                                           const Grid& goal) override;

  bool HasForcedNeighbours(const Grid& new_point, const Grid& next_point,
                           const Grid& motion) const;

  Grid jump(const Grid& new_point, const Grid& motion, const int id);

 private:
  std::priority_queue<Grid, std::vector<Grid>, compare_cost> open_list_;
  std::vector<Grid> closed_list_;
  std::unordered_set<int> pruned;
  Grid start_, goal_;
};
#endif  // JUMP_POINT_SEARCH_H
