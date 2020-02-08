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

struct CarState {
  double x;
  double y;
  double s; 
  double d;
  double yaw;
  double speed;
};

struct Path {
  double x;
  double y;
  double d;
  double s;
};

struct Map {
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
};

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
   * @param speed_limit speed_limit of the road [mph]
   */
  void init(const Map &map, size_t number_lanes, double width_lane, double speed_limit);

  /**
   * Set the current location of the car given by localization module (external)
   * @param loc The location (and velocity)
   */
  void set_location(const CarState & loc);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }


  Map map;
 private:
  // Flag, if filter is initialized
  bool is_initialized;
  size_t number_lanes;
  double width_lane;
  double speed_limit;
  CarState location;
};

#endif  // PATH_PLANNER_H_