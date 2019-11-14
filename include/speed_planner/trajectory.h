#ifndef TRAJECTORY_ROS_H
#define TRAJECTORY_ROS_H 

#include <vector>
#include <cassert>

class Trajectory
{
public:
    Trajectory() {};
    Trajectory(const std::vector<double> current_trajectory_x, 
               const std::vector<double> current_trajectory_y,
               const std::vector<double>& calculated_velocity)
    {
        assert(current_trajectory_x.size() == calculated_velocity.size());
        assert(current_trajectory_y.size() == calculated_velocity.size());

        x_.resize(current_trajectory_x.size());
        y_.resize(current_trajectory_y.size());
        velocity_.resize(calculated_velocity.size());

        for(int i=0; i<current_trajectory_x.size(); ++i)
        {
            x_[i] = current_trajectory_x[i];
            y_[i] = current_trajectory_y[i];
        }

        for(int i=0; i<velocity_.size(); ++i)
            velocity_[i] = calculated_velocity[i];
    }

    ~Trajectory() = default;

    void set(const TrajectoryLoader& current_trajectory, const std::vector<double>& calculated_velocity)
    {
        assert(current_trajectory.x_.size() == calculated_velocity.size());
        assert(current_trajectory.y_.size() == calculated_velocity.size());

        x_.resize(current_trajectory.x_.size());
        y_.resize(current_trajectory.y_.size());
        velocity_.resize(calculated_velocity.size());

        for(int i=0; i<current_trajectory.x_.size(); ++i)
        {
            x_[i] = current_trajectory.x_[i];
            y_[i] = current_trajectory.y_[i];
        }

        for(int i=0; i<velocity_.size(); ++i)
            velocity_[i] = calculated_velocity[i];
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> velocity_;
};

#endif //TRAJECTORY_ROS_H