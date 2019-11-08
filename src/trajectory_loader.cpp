#include "speed_planner/trajectory_loader.h"

TrajectoryLoader::TrajectoryLoader(const std::vector<double> &x,
                                   const std::vector<double> &y,
                                   const double ds,
                                   const double required_length,
                                   const int skip_size,
                                   const int smooth_size)
                                   :ds_(ds),
                                    trajectory_length_(0.0),
                                    data_size_(-1),
                                    size_(-1),
                                    smooth_size_(smooth_size),
                                    spline_x_(),
                                    spline_y_()

{
    double px = 0.0;
    double py = 0.0;

    for(size_t i=0; i<x.size(); ++i)
    {
        double cx = x[i];
        double cy = y[i];

        if(i>0)
        {
            double dx = cx - px;
            double dy = cy - py;
            trajectory_length_ += std::sqrt(dx*dx + dy*dy);
        }

        if(i%skip_size == 0)
        {
            data_x_.push_back(cx);
            data_y_.push_back(cy);
            s_.push_back(trajectory_length_);
            if(trajectory_length_>required_length)
                break;
        }

        px = cx;
        py = cy;
    }

    data_size_ = data_x_.size();

    set_spline();
    set_waypoints();
}

bool TrajectoryLoader::set_spline()
{
    if(data_size_ == -1)
        return false;

    spline_x_.set_points(s_, data_x_);
    spline_y_.set_points(s_, data_y_);

    return true;
}

void TrajectoryLoader::set_waypoints()
{
    int required_waypoint_size = int(trajectory_length_/ds_);
    x_.reserve(required_waypoint_size);
    y_.reserve(required_waypoint_size);
    yaw_.reserve(required_waypoint_size);
    curvature_.reserve(required_waypoint_size);

    for(int i=0; i<required_waypoint_size; ++i)
    {
        double waypoint_x = spline_x_(i*ds_);
        double waypoint_y = spline_y_(i*ds_);
        double d_x  = spline_x_.deriv(tk::spline::first_deriv, i*ds_);
        double d_y  = spline_y_.deriv(tk::spline::first_deriv, i*ds_);
        double dd_x = spline_x_.deriv(tk::spline::second_deriv, i*ds_);
        double dd_y = spline_y_.deriv(tk::spline::second_deriv, i*ds_);
        double yaw= std::atan2(d_y, d_x);
        double curvature = std::fabs(dd_y*d_x-dd_x*d_y)/std::pow((d_x*d_x+d_y*d_y),(1.5));
        x_.push_back(waypoint_x);
        y_.push_back(waypoint_y);
        yaw_.push_back(yaw);
        curvature_.push_back(curvature);
    }

    for(size_t i=0; i<curvature_.size(); ++i)
    {
        double sum=0.0;
        int size = 0;
        for(int j=-smooth_size_; j<smooth_size_; ++j)
        {
            if(i+j<0 || i+j>curvature_.size())
                continue;
            sum += curvature_[i+j];
            size++;
        }
        if(size<=0)
            continue;
        curvature_[i] = sum/(size);
    }

    size_ = x_.size();
}