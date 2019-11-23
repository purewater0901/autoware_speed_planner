#include "speed_planner/obstacle.h"

Obstacle::Obstacle(const double x, 
                   const double y, 
                   const double angle, 
                   const double radius) 
                   : x_(x), y_(y), angle_(angle), radius_(radius)
{
}

StaticObstacle::StaticObstacle(const double x,
                               const double y,
                               const double angle,
                               const double radius)
                               : Obstacle(x, y, angle, radius)
{
    type_ = STATIC;
}

std::vector<std::pair<double, std::pair<double, double>>> StaticObstacle::getPosition()
{
    std::vector<std::pair<double, std::pair<double, double>>> position;
    position.resize(1);

    position[0].first = 0.0;
    position[0].second.first  = x_;
    position[1].second.second = y_;

    return position;
}

DynamicObstacle::DynamicObstacle(const double x,
                                 const double y,
                                 const double angle,
                                 const double radius,
                                 const double translational_velocity,
                                 const double predicted_time_horizon,
                                 const double dt)
                                : Obstacle(x, y, angle, radius), 
                                  translationa_velocity_(translational_velocity),
                                  predicted_time_horizon_(predicted_time_horizon),
                                  dt_(dt)
{
    type_ = DYNAMIC;
    setFutureTrajectory();
}

void DynamicObstacle::setFutureTrajectory()
{
    int N = int(predicted_time_horizon_/dt_);
    predicted_trajectory_.resize(N);

    predicted_trajectory_[0].first = 0.0;
    predicted_trajectory_[0].second.first  = x_;
    predicted_trajectory_[0].second.second = y_;
    for(int i=1; i<N; ++i)
    {
        predicted_trajectory_[i].first = predicted_trajectory_[i-1].first + dt_;
        predicted_trajectory_[i].second.first  = predicted_trajectory_[i-1].second.first  + translationa_velocity_*cos(angle_)*dt_;
        predicted_trajectory_[i].second.second = predicted_trajectory_[i-1].second.second + translationa_velocity_*sin(angle_)*dt_;
    }
}

std::vector<std::pair<double, std::pair<double, double>>> DynamicObstacle::getPosition()
{
    return predicted_trajectory_;
}