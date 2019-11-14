#ifndef UTILS_H
#define UTILS_H 
#include <vector>
#include <memory>
#include <cmath>
#include <limits>

int getNearestId(const double current_x,
                 const double current_y,
                 const std::vector<double>& trajectory_x,
                 const std::vector<double>& trajectory_y,
                 const int ini_id = 0);

#endif