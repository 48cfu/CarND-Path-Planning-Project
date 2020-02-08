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

void PathPlanner::init(size_t number_lanes, double width_lane, double speed_limit){
  if (!initialized()){
    this->number_lanes = number_lanes;
    this->width_lane = width_lane;
    this->speed_limit = speed_limit;
  }
}