#ifndef SPEED_PLANNER_VEHICLE_INFO_H
#define SPEED_PLANNER_VEHICLE_INFO_H

#include <iostream>
#include <vector>
#include <cmath>

class VehicleInfo
{
public:
    VehicleInfo(const double length, const double width, const double wheelBase, const double safety_distance)
    : wheel_base_(wheelBase), safety_distance_(safety_distance)
    {
        if(length>width)
        {
            length_ = length;
            width_ = width;
        }
        else
        {
            length_ = width;
            width_ = length;
        }

        circumcircle_radius_ = std::sqrt(std::pow(length_/2.0, 2) + std::pow(width_/2.0, 2));    // circumcircle radius
        middlecircle_radius_ = std::sqrt((width_/4)*(width_/4)+(length_/4)*(length_/4));
        footprintcircle_radius_ = std::sqrt(std::pow(width_/4,2)+std::pow(length_/8,2));

        middlecircle_deviation_ = length_/8;
        footprintcircle_deviation_ = std::sqrt(std::pow(width_/4,2)+std::pow(3*length_/8, 2));

        footprint_deviation_yaw_ = std::atan2(3*length_/8, width_/4);
    }
    ~VehicleInfo() = default;

    double length_;
    double width_;
    double wheel_base_;
    double safety_distance_;
    double circumcircle_radius_;
    double middlecircle_radius_;
    double footprintcircle_radius_;
    double middlecircle_deviation_;
    double footprintcircle_deviation_;
    double footprint_deviation_yaw_;
};

#endif
