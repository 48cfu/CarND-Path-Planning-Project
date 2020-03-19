#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double calculate_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions);

double inefficiency_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions);

#endif // COST_H