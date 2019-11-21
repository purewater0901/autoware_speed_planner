#ifndef TRAJECTORY_ROS_H
#define TRAJECTORY_ROS_H 

#include <vector>
#include <cassert>

class Trajectory
{
public:
    Trajectory() {};
    Trajectory(const std::vector<double>& current_trajectory_x, 
               const std::vector<double>& current_trajectory_y,
               const std::vector<double>& current_trajectory_yaw,
               const std::vector<double>& current_trajectory_curvature,
               const std::vector<double>& calculated_velocity, 
               const std::vector<double>& calculated_acceleration)
    {
        assert(current_trajectory_x.size() == calculated_velocity.size());
        assert(current_trajectory_y.size() == calculated_velocity.size());
        assert(current_trajectory_yaw.size() == calculated_velocity.size());
        assert(current_trajectory_curvature.size() == calculated_velocity.size());
        assert(calculated_acceleration.size() == calculated_velocity.size());

        x_.resize(current_trajectory_x.size());
        y_.resize(current_trajectory_y.size());
        yaw_.resize(current_trajectory_y.size());
        curvature_.resize(current_trajectory_y.size());
        velocity_.resize(calculated_velocity.size());
        acceleration_.resize(calculated_acceleration.size());

        for(int i=0; i<current_trajectory_x.size(); ++i)
        {
            x_[i] = current_trajectory_x[i];
            y_[i] = current_trajectory_y[i];
            yaw_[i] = current_trajectory_yaw[i];
            curvature_[i] = current_trajectory_curvature[i];
        }

        for(int i=0; i<velocity_.size(); ++i)
        {
            velocity_[i] = calculated_velocity[i];
            acceleration_[i] = calculated_acceleration[i];
        }
    }

    Trajectory(const std::vector<double>& current_trajectory_x, 
               const std::vector<double>& current_trajectory_y,
               const std::vector<double>& current_trajectory_yaw,
               const std::vector<double>& current_trajectory_curvature,
               const int nearest_id)
    {
        int required_size = current_trajectory_x.size()-nearest_id;
        x_.reserve(required_size);
        y_.reserve(required_size);
        yaw_.reserve(required_size);
        curvature_.reserve(required_size);

        for(int i=nearest_id; i<current_trajectory_x.size(); ++i)
        {
            x_.push_back(current_trajectory_x[i]);
            y_.push_back(current_trajectory_y[i]);
            yaw_.push_back(current_trajectory_yaw[i]);
            curvature_.push_back(current_trajectory_curvature[i]);
        }
    }


    ~Trajectory() = default;

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> yaw_;
    std::vector<double> curvature_;
    std::vector<double> velocity_;
    std::vector<double> acceleration_;
};

#endif //TRAJECTORY_ROS_H