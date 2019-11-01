#pragma once

#include <vector>
#include "speed_planner/cubic_spline.h"

class ReferenceTrajectory
{
    public:
        ReferenceTrajectory(const std::vector<double>& x, 
                             const std::vector<double>& y, 
                             const double ds, 
                             const int N) : spline_(x, y)
    {
        x_.reserve(spline_.s.size());
        y_.reserve(spline_.s.size());
        yaw_.reserve(spline_.s.size());
        curvature_.reserve(spline_.s.size());

        for(float s=0; s<spline_.s.back(); s+=ds)
        {
            std::array<double, 2> point = spline_.calc_position(s);

            x_.push_back(point[0]);
            y_.push_back(point[1]);
            yaw_.push_back(spline_.calc_yaw(s));
            curvature_.push_back(spline_.calc_curvature(s));

            if(x_.size() > N)
                break;
        }
    }

    int size()
    {
        return x_.size();
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> yaw_;
    std::vector<double> curvature_;
    Spline2D spline_;
};