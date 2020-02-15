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
    this->ref_velocity = 49.5;
  }
}

void PathPlanner::set_location(const CarState & car_location){
  this->car_location = car_location;
}

vector<vector<double>> PathPlanner::keep_lane(const CarState & location, const Path & previous_path){
  set_location(location);

  int prev_size = previous_path.x.size();

  //something here
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
    ptsx.push_back(car_location.x - cos(car_location.yaw));
    ptsx.push_back(car_location.x);

    ptsy.push_back(car_location.y - sin(car_location.yaw));
    ptsy.push_back(car_location.y);
  } else {
    // use the previous ptaht's end point as starting reference
    ref_x = previous_path.x[prev_size - 1];
    ref_y = previous_path.y[prev_size - 1];

    double ref_x_prev = previous_path.x[prev_size - 2];
    double ref_y_prev = previous_path.y[prev_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use 2 points that make the path tangent ot the car
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
    
  }
  // In Frenet add  evenly 30m spaced points  ahead of the starting reference
  vector<double> next_wp0 = addXFrenet(30);
  vector<double> next_wp1 = addXFrenet(60);
  vector<double> next_wp2 = addXFrenet(90);

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

    ptsx[i] = shitf_x * cos(0 - ref_yaw) - shitf_y * sin(0 - ref_yaw);
    ptsy[i] = shitf_x * sin(0 - ref_yaw) + shitf_y * cos(0 - ref_yaw);
  }
  // create a spline
  tk::spline spline;
  // set (x, y) points of the spline
  spline.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with all the previous path points from the last time
  for (size_t i = 0; i < previous_path.x.size(); i++) {
    next_x_vals.push_back(previous_path.x[i]);
    next_y_vals.push_back(previous_path.y[i]);
  }

  // Calculate how to break up  spline points so that we travel at the desired reference velocity
  double target_x = 30;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

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
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * sin(ref_yaw);

    x_point += x_ref;
    y_point += y_ref;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  std::cout << "size next_x_vay = " << next_x_vals.size() << std::endl << std::flush;
  vector<vector<double>> next_xy;
  next_xy.push_back(next_x_vals);
  next_xy.push_back(next_y_vals);

  return next_xy;
}