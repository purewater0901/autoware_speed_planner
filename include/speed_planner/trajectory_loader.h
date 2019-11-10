#ifndef SPEED_PLANNER_TRAJECTORY_LOADER_H
#define SPEED_PLANNER_TRAJECTORY_LOADER_H

#include <vector>
#include <cmath>
#include "speed_planner/spline.h"

class TrajectoryLoader
{
public:
    TrajectoryLoader(const std::vector<double>& x,
                     const std::vector<double>& y,
                     const double ds,
                     const double required_length,
                     const int skip_size,
                     const int smooth_size);
    ~TrajectoryLoader() = default;
    void set_waypoints();
    int size(){ return size_; }
    double get_ds(){ return ds_; }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> yaw_;
    std::vector<double> curvature_;

private:
    std::vector<double> s_;
    std::vector<double> data_x_;
    std::vector<double> data_y_;
    int data_size_;
    int size_;
    int smooth_size_;
    double trajectory_length_;
    double ds_;
    tk::spline spline_x_;
    tk::spline spline_y_;

    bool set_spline();

};

#endif //SPEED_PLANNER_TRAJECTORY_LOADER_H
