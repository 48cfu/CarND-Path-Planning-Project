#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

using namespace std;

const double EFFICIENCY = pow(10, 4);


double calculate_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions) {
  double cost = 0.0;

  cost += inefficiency_cost(vehicle, trajectory, predictions);

  return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions) {

  map<string, int> lane_direction = {{"PLCL", -1}, {"PLCR", 1}};

  int final_lane = trajectory.lane;
  int intended_lane = trajectory.lane;
  if (lane_direction.find(trajectory.FSM_state) != lane_direction.end() ) {
    intended_lane = intended_lane + lane_direction[trajectory.FSM_state];
  }

  // these are actually static members vehicle.get_kinematics()
  vector<double> intended_kinematics = vehicle.get_kinematics(vehicle, intended_lane, 40.0, predictions);
  vector<double> final_kinematics = vehicle.get_kinematics(vehicle, final_lane, 40.0, predictions);

  double intended_speed = intended_kinematics[0];
  double final_speed = final_kinematics[0];

  // Don't jump between lanes if it's just slightly faster.
  if (abs(intended_speed - final_speed) < 1.5) {
    final_speed = intended_speed;
  }

  double cost = EFFICIENCY * (2.0*22.2 - intended_speed - final_speed)/22.2;

  return cost;
}
