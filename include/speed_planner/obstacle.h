#ifndef SPEED_PLANNER_OBJECTS_H
#define SPEED_PLANNER_OBJECTS_H

class Obstacle
{
public:
    Obstacle(const double x, 
             const double y, 
             const double angle, 
             const double radius, 
             const double translational_velocity)
             :x_(x), y_(y), angle_(angle), radius_(radius), translational_velocity_(translational_velocity)
             {};

    Obstacle() {};

    double x_;
    double y_;
    double angle_;
    double radius_;
    double translational_velocity_;
};


#endif
