#ifndef SPEED_PLANNER_OBJECTS_H
#define SPEED_PLANNER_OBJECTS_H

#include <vector>
#include <cmath>

class Obstacle
{
public:
    enum TYPE
    {
        STATIC  = 0,
        DYNAMIC = 1,
    };

    Obstacle(const double x, 
             const double y, 
             const double angle, 
             const double radius);

    virtual std::vector<std::pair<double, std::pair<double, double>>> getPosition() = 0;

    TYPE getType() { return type_; }
    double getRadius() { return radius_; }


    double x_;
    double y_;
    double angle_;
    double radius_;
    TYPE type_;
};

class StaticObstacle : public Obstacle
{
public:
    StaticObstacle(const double x,
                   const double y,
                   const double angle,
                   const double radius);

    std::vector<std::pair<double, std::pair<double, double>>> getPosition();
};

class DynamicObstacle : public Obstacle
{
public:
    DynamicObstacle(const double x,
                    const double y,
                    const double angle,
                    const double radius,
                    const double translational_velocity,
                    const double predicted_time_horizon,
                    const double dt);

    void setFutureTrajectory();
    std::vector<std::pair<double, std::pair<double, double>>> getPosition();

private:
    double predicted_time_horizon_;
    double dt_;
    double translationa_velocity_;
    std::vector<std::pair<double, std::pair<double, double>>> predicted_trajectory_;
};

#endif
