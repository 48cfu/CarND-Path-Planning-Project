#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "helpers.h"

using namespace std;

Vehicle::Vehicle(){}

Vehicle::Vehicle(double id, int lane, CarState car_state, string FSM_state) {
  this->id = id;
  this->lane = lane;
  this->car_state = car_state;
  this->FSM_state = FSM_state;
  this->speed_limit = 22.2;
}

Vehicle::~Vehicle() {}

Vehicle Vehicle::choose_next_state(vector<Vehicle> predictions) {
  vector<string> successors = successor_states();

  Vehicle best_trajectory;
  double min_cost = numeric_limits<double>::max();
  for (int i = 0; i < successors.size(); i++) {
    string state = successors[i];

    Vehicle trajectory = generate_trajectory(state, predictions);

    double cost = calculate_cost(*this, trajectory, predictions);

    if (cost < min_cost) {
      min_cost = cost;
      best_trajectory = trajectory;
    }
  }

  return best_trajectory;
}

vector<string> Vehicle::successor_states() {
  
  vector<string> states;
  states.push_back("KL");

  string state = this->FSM_state;
  if (state.compare("KL") == 0) {
    if (this->lane > 0) {
      states.push_back("PLCL");
    }
    if (this->lane < 2) {
      states.push_back("PLCR");
    }
  }
  if (state.compare("PLCL") == 0) {
    if (this->lane > 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  if (state.compare("PLCR") == 0) {
    if (this->lane < 2) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

Vehicle Vehicle::generate_trajectory(string state, vector<Vehicle> predictions) {
  Vehicle trajectory;
  if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

Vehicle Vehicle::keep_lane_trajectory(vector<Vehicle> predictions) {
  vector<double> lane_kinematics = get_kinematics(*this, this->lane, 20.0, predictions);
  double speed = lane_kinematics[0];
  CarState car_state = this->car_state;
  car_state.speed = speed;
  return Vehicle(this->id, this->lane, car_state, "KL");
}

Vehicle Vehicle::prep_lane_change_trajectory(string FSMstate, vector<Vehicle> predictions) {
  
  int next_lane = this->lane + this->lane_direction[FSMstate];

  vector<double> curr_lane_kinematics = get_kinematics(*this, this->lane, 20.0, predictions);
  vector<double> next_lane_kinematics = get_kinematics(*this, next_lane, 20.0, predictions);

  vector<double> best_kinematics;
  if (curr_lane_kinematics[0] < next_lane_kinematics[0]) {
    best_kinematics = curr_lane_kinematics;
  } else {
    best_kinematics = next_lane_kinematics;
  }

  double speed = best_kinematics[0];
  CarState car_state = this->car_state;
  car_state.speed = speed;

  return Vehicle(this->id, this->lane, car_state, FSMstate);
}

Vehicle Vehicle::lane_change_trajectory(string FSMstate, vector<Vehicle> predictions) {
 
  int next_lane = this->lane + this->lane_direction[FSMstate];

  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];

    if (next_lane == pred.lane) { //in the target lane
      if (this->car_state.s < pred.car_state.s + 3 && this->car_state.s > pred.car_state.s - 33) { // Car blocking the lane
        return this->keep_lane_trajectory(predictions);
      }
    }
  }

  vector<double> next_lane_kinematics = get_kinematics(*this, next_lane, 20.0, predictions);
  double speed = next_lane_kinematics[0];

  CarState car_state = this->car_state;
  car_state.speed = speed;

  return Vehicle(this->id, next_lane, car_state, FSMstate);
}

vector<double> Vehicle::get_kinematics(const Vehicle & vehicle, int lane, double distance_ahead, vector<Vehicle> predictions) const {
  
  double lane_speed = mps2MPH(this->speed_limit);

  vector<Vehicle> vehicle_ahead = get_vehicle_ahead(vehicle, lane, predictions);
  if (vehicle_ahead.size() > 0) {
    // slightly slower than the vehicle in front
    if (vehicle_ahead[0].car_state.s - vehicle.car_state.s < distance_ahead) { // slow down
      lane_speed = 0.95 * vehicle_ahead[0].car_state.speed; 
    }
  }

  return {lane_speed};
}

vector<Vehicle> Vehicle::get_vehicle_behind(const Vehicle & vehicle, int lane, vector<Vehicle> predictions) const {
  
  vector<Vehicle> vehicle_behind;

  double min_dist = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    // same lane and behind
    if (pred.lane == lane && vehicle.car_state.s > pred.car_state.s) { 
      double dist = vehicle.car_state.s - pred.car_state.s;
      if (dist < min_dist) {
        min_dist = dist;
        vehicle_behind = {pred};
      }
    }
  }

  return vehicle_behind;
}

vector<Vehicle> Vehicle::get_vehicle_ahead(const Vehicle & vehicle, int lane, vector<Vehicle> predictions) const{
  
  vector<Vehicle> vehicle_ahead;

  double min_dist = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    // same lane and in front
    if (pred.lane == lane && vehicle.car_state.s < pred.car_state.s) { 
      double dist = pred.car_state.s - vehicle.car_state.s;
      if (dist < min_dist) {
        min_dist = dist;
        vehicle_ahead = {pred};
      }
    }
  }

  return vehicle_ahead;
}