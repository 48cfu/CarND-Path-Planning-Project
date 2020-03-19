#ifndef COST_H
#define COST_H
#include "vehicle.h"

double calculate_cost(const Vehicle & vehicle, const Vehicle & trajectory, const std::vector<Vehicle> & predictions);

double inefficiency_cost(const Vehicle & vehicle, const Vehicle & trajectory, const std::vector<Vehicle> & predictions);

#endif // COST_H