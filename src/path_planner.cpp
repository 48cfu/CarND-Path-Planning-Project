/**
 * path_planner.cpp
 *
 * Created on: Feb 08, 2020
 * Author: 48cfu
 */

#include "path_planner.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helpers.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::string;
using std::vector;

void PathPlanner::init(size_t current_lane, const Map &map, size_t number_lanes, double width_lane, double speed_limit){
  if (!initialized()){
    this->number_lanes = number_lanes;
    this->width_lane = width_lane;
    this->speed_limit = speed_limit;
    is_initialized = true;
    this->map = map;
    this->current_lane = current_lane;
    this->ref_velocity = 2;
    this->max_velocity_acceleration = 1.8 * 0.224; //about 10 m/s in velocity
    this->safety_distance = 30.0;
  }
}

void PathPlanner::set_location(const CarState & car_location){
  this->car_location = car_location;
}

vector<vector<double>> PathPlanner::keep_lane(const CarState & location, const Path & previous_path, vector<vector<double>>  sensor_fusion){
  set_location(location);

  int prev_size = previous_path.x.size();

  if (prev_size > 0) {
    car_location.s = previous_path.s;
  }

  bool too_close = false;
  double check_speed = 0;
  double check_car_s = 0;
  // find reference velocity to use
  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    double d = sensor_fusion[i][6];
    if ((2 + 4*current_lane - 2) < d  && d < (2 + 4*current_lane + 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      check_speed = sqrt(vx * vx + vy * vy);
      check_car_s = sensor_fusion[i][5];

      // if using previous points can project s  value out
      check_car_s += ((double) prev_size * .02 * check_speed);

      // check s values greater than mine and s gap
      if (car_location.s < check_car_s && check_car_s < this->safety_distance + car_location.s) {
        too_close = true;
        break;
      }
    } /*else if (car is changing to my lane) {
      too_close = true;
    }*/
  }
  bool previous_too_close = false;
  if (too_close) {
    if (previous_too_close){
      // continue descreasing until safety distance is met.
      // this because someone might have done a dangerous manoveur, and 
      // they are now in front of us
      ref_velocity = std::min(ref_velocity - this->max_velocity_acceleration, 5.0);
    } else {
      ref_velocity = std::max(ref_velocity - this->max_velocity_acceleration, mps2MPH(check_speed));
    }
    //std::cout << "vel = " << check_speed << std::endl << std::flush;
    // do lane change
  } else {
    ref_velocity = std::min(ref_velocity + this->max_velocity_acceleration, speed_limit - 0.2);
  }

  previous_too_close = too_close;
  // create a list of widely spread (x,y) waypoints, evenly spaced at 30m
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states
  double ref_x = car_location.x;
  double ref_y = car_location.y;
  double ref_yaw = deg2rad(car_location.yaw);

  // if the previous path size is almost empty, use the current
  // car_location as a starting reference
  if (prev_size < 2) {
    // use 2 points that make the path tangent ot the car
    ptsx.push_back(car_location.x - std::cos(car_location.yaw));
    ptsx.push_back(car_location.x);

    ptsy.push_back(car_location.y - std::sin(car_location.yaw));
    ptsy.push_back(car_location.y);
  } else {
    // use the previous ptaht's end point as starting reference
    ref_x = previous_path.x[prev_size - 1];
    ref_y = previous_path.y[prev_size - 1];

    double ref_x_prev = previous_path.x[prev_size - 2];
    double ref_y_prev = previous_path.y[prev_size - 2];

    ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use 2 points that make the path tangent ot the car
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
    
  }
  // In Frenet add  evenly 30m spaced points  ahead of the starting reference
  vector<double> next_wp0 = addXFrenet(30.0);
  vector<double> next_wp1 = addXFrenet(60.0);
  vector<double> next_wp2 = addXFrenet(90.0);

  ptsx.push_back(next_wp0[0]);
  ptsy.push_back(next_wp0[1]);
  
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);

  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  // Transformation into car-coordinate
  for (size_t i = 0; i < ptsx.size(); i++) {
    //shift car reference angle to 0 degrees
    double shitf_x = ptsx[i] - ref_x;
    double shitf_y = ptsy[i] - ref_y;

    ptsx[i] = shitf_x * std::cos(0 - ref_yaw) - shitf_y * std::sin(0 - ref_yaw);
    ptsy[i] = shitf_x * std::sin(0 - ref_yaw) + shitf_y * std::cos(0 - ref_yaw);
  }
  // create a spline
  tk::spline spline;
  // set (x, y) points of the spline
  spline.set_points(ptsx, ptsy);

  // define the actual (x, y) we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with all the previous path points from the last time
  for (size_t i = 0; i < previous_path.x.size(); i++) {
    next_x_vals.push_back(previous_path.x[i]);
    next_y_vals.push_back(previous_path.y[i]);
  }

  // Calculate how to break up  spline points so that we travel at the desired reference velocity
  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  // Fill up the rest of our path planner  after filling it with the previous points, 
  // here we will always ouptut 50 points
  for (size_t i = 0; i <= 50 - previous_path.x.size(); i++) {
    double N =  target_dist / (0.02 * MPH2mps(ref_velocity));
    
    double x_point = x_add_on + (target_x / N);
    double y_point = spline(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Trasformation to map coordinate
    x_point = x_ref * std::cos(ref_yaw) - y_ref * std::sin(ref_yaw);
    y_point = x_ref * std::sin(ref_yaw) + y_ref * std::cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  //std::cout << "size next_x_vay = " << next_x_vals.size() << std::endl << std::flush;
  vector<vector<double>> next_xy;
  next_xy.push_back(next_x_vals);
  next_xy.push_back(next_y_vals);

  return next_xy;
}

/**
   * Calculates the Jerk Minimizing Trajectory that connects the initial state
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
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
vector<double> PathPlanner::JMT(vector<double> &start, vector<double> &end, double T) {
   double si = start[0];
   double dot_si = start[1];
   double ddot_si = start[2];
   
   double sf = end[0];
   double dot_sf = end[1];
   double ddot_sf = end[2];
   
    MatrixXd T_matrix = MatrixXd(3, 3);
    T_matrix <<   pow(T, 3), pow(T, 4), pow(T, 5),
                  3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
                  6*pow(T, 1), 12*pow(T, 2), 20*pow(T, 3);
    VectorXd s_vector = VectorXd(3);
    s_vector <<   sf - (si + dot_si*T + 0.5*ddot_si * T * T),
                  dot_sf - (dot_si + ddot_si * T),
                  ddot_sf - ddot_si;

    double a0 = si;
    double a1 = dot_si;
    double a2 = 0.5 * ddot_si;

    VectorXd a345 = T_matrix.inverse() * s_vector;
    
  return {a0, a1, a2, a345[0], a345[1], a345[2]};
}