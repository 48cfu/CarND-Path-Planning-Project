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
    this->ref_velocity = 0;
  }
}

void PathPlanner::set_location(const CarState & loc){
  this->location = loc;
}

vector<vector<double>> PathPlanner::keep_lane(const Path & previous_path){
  int prev_size = previous_path.x.size();

  //something here
  // create a list of widely spread (x,y) waypoints, evenly spaced at 30m
  vector<double> ptsx;
  vector<double> ptsy;

  vector<vector<double>> next_xy;
  return next_xy;
}