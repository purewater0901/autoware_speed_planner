#include "speed_planner/utils.h"

int getNearestId(const double current_x,
                 const double current_y,
                 const std::vector<double>& trajectory_x,
                 const std::vector<double>& trajectory_y,
                 const int ini_id)
{
    int min_id = -1;
    double min_distance = std::numeric_limits<double>::max();
    if(ini_id>trajectory_x.size())
        return trajectory_x.size()-1;

    for(size_t i=ini_id; i<trajectory_x.size(); ++i)
    {
        double tmp_dis = std::sqrt(std::pow(current_x-trajectory_x[i], 2)+std::pow(current_y-trajectory_y[i], 2));
        if(tmp_dis<min_distance)
        {
            min_id = i;
            min_distance = tmp_dis;
        }
    }

    if(min_id==-1)
        min_id = 0;

    return min_id;
}