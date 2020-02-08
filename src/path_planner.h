/**
 * path_planner.h
 *
 * Created on: Feb 08, 2020
 * Author: 48cfu
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <string>
#include <vector>
#include "helpers.h"


class PathPlanner {  
 public:
  // Constructor
  PathPlanner() : is_initialized(false) {}

  // Destructor
  ~PathPlanner() {}

  /**
   * init Initializes the path planner with description of the road
   *   distribution around first position and all the weights to 1.
   * @param number_lanes numer of lanes
   * @param width_lane width of each lane [m]
   * @param speed_limit speed_limit of the road [m/s]
   */
  void init(size_t number_lanes, double width_lane, double speed_limit);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }

 private:
  // Flag, if filter is initialized
  bool is_initialized;
  size_t number_lanes;
  double width_lane;
  double speed_limit;
};

#endif  // PATH_PLANNER_H_