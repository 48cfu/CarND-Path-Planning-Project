#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "helpers.h"

class Vehicle {
public:

  std::map<string, int> lane_direction = {{"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  double id;
  int lane;
  CarState car_state;
  string FSM_state;
  double speed_limit; 

  Vehicle();
  Vehicle(double id, int lane, CarState car_state, string FSM_state = "KL");
  virtual ~Vehicle();

  std::vector<string> successor_states();
  Vehicle choose_next_state(vector<Vehicle> predictions);
  Vehicle generate_trajectory(string state, std::vector<Vehicle> predictions);
  Vehicle keep_lane_trajectory(vector<Vehicle> predictions);
  Vehicle prep_lane_change_trajectory(string state, vector<Vehicle> predictions);
  Vehicle lane_change_trajectory(string state, vector<Vehicle> predictions);
  std::vector<Vehicle> get_vehicle_ahead(const Vehicle & vehicle, int lane, std::vector<Vehicle> predictions) const;
  std::vector<Vehicle> get_vehicle_behind(const Vehicle & vehicle, int lane, std::vector<Vehicle> predictions) const;
  std::vector<double> get_kinematics(const Vehicle & vehicle, int lane, double distance_ahead, std::vector<Vehicle> predictions) const;

};

#endif // VEHICLE_H