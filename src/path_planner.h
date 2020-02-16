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
  vector<double> x;
  vector<double> y;
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
  void init(size_t current_lane, const Map &map, size_t number_lanes, double width_lane, double speed_limit);

  /**
   * Set the current location of the car given by localization module (external)
   * @param loc The location (and velocity)
   */
  void set_location(const CarState & car_location);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }

  /**
   * Keep lane
   */
  vector<vector<double>> keep_lane(const CarState & car_location, const Path & previous_path, vector<vector<double>> sensor_fusion);
  
  /**
   * In Frenet add X meters to car's current location
   */
  vector<double> addXFrenet(double X){
    return getXY(car_location.s + X, 2 + 4 * current_lane, map.waypoints_s, map.waypoints_x, map.waypoints_y);
  }

  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   */
  vector<double> JMT(vector<double> &start, vector<double> &end, double T);

  Map map;
 private:
  // Flag, if filter is initialized
  bool is_initialized;
  size_t number_lanes;
  double width_lane;
  double speed_limit;
  CarState car_location;
  int current_lane;
  double ref_velocity;
  double max_velocity_acceleration;
  double safety_distance;
};

#endif  // PATH_PLANNER_H_