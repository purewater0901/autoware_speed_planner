#include "speed_planner/collision_checker.h"

bool CollisionChecker::check(Trajectory& trajectory, 
                             const std::vector<Obstacle>& obstacles,
                             const std::unique_ptr<VehicleInfo>& ego_vehicle,
                             std::pair<double, double>& result)
{
    if(obstacles.empty())
        return false;

    int check_interval_size = 1.0/0.1;
    for(Obstacle obstacle : obstacles)
    {
        for(size_t i=0; i<trajectory.x_.size(); i+=check_interval_size)
        {
            //step1 check the largest radius around the vehicle
            double x = trajectory.x_[i];  //vehicle center's x coordinate 
            double y = trajectory.y_[i];  //vehicle center's y coordinate
            double yaw = trajectory.yaw_[i];
            double dist = std::sqrt(std::pow((x - obstacle.x_), 2) + std::pow((y - obstacle.y_), 2));
            double clearance_radius = ego_vehicle->circumcircle_radius_+ego_vehicle->safety_distance_; //largest_circle + safety_distance

            if(dist <= obstacle.radius_+ clearance_radius)
            {
                //step2 check the middle circles
                double middle_clearance_radius = ego_vehicle->middlecircle_radius_ + ego_vehicle->safety_distance_;
                for(size_t mcircleId=0; mcircleId<2; ++mcircleId)
                {
                    double xmc = x + ego_vehicle->middlecircle_deviation_*cos(yaw+mcircleId*M_PI);
                    double ymc = y + ego_vehicle->middlecircle_deviation_*sin(yaw+mcircleId*M_PI);
                    double middleDist = std::sqrt(std::pow(xmc-obstacle.x_, 2)+std::pow(ymc-obstacle.y_, 2));

                    if(middleDist <= obstacle.radius_ + middle_clearance_radius)
                    {
                        std::cout << "Collide with Middle Circle" << std::endl;
                        return true; //collision
                    }
                }

                //step3 check the 4 footprint circles
                double footprint_clearance_radius = ego_vehicle->footprintcircle_radius_
                                                   + ego_vehicle->safety_distance_;
                for(size_t fcircleId=0; fcircleId<4; ++fcircleId)
                {
                    double xfc;
                    double yfc;
                    if(fcircleId==0)
                    {
                        xfc = x + ego_vehicle->footprintcircle_deviation_*cos(yaw-M_PI/2+ego_vehicle->footprint_deviation_yaw_);
                        yfc = y + ego_vehicle->footprintcircle_deviation_*sin(yaw-M_PI/2+ego_vehicle->footprint_deviation_yaw_);
                    }
                    else if(fcircleId==1)
                    {
                        xfc = x + ego_vehicle->footprintcircle_deviation_*cos(yaw+M_PI/2-ego_vehicle->footprint_deviation_yaw_);
                        yfc = y + ego_vehicle->footprintcircle_deviation_*sin(yaw+M_PI/2-ego_vehicle->footprint_deviation_yaw_);
                    }
                    else if(fcircleId==2)
                    {
                        xfc = x + ego_vehicle->footprintcircle_deviation_*cos(yaw+M_PI/2+ego_vehicle->footprint_deviation_yaw_);
                        yfc = y + ego_vehicle->footprintcircle_deviation_*sin(yaw+M_PI/2+ego_vehicle->footprint_deviation_yaw_);
                    }
                    else if(fcircleId==3)
                    {
                        xfc = x + ego_vehicle->footprintcircle_deviation_*cos(yaw+3*M_PI/2-ego_vehicle->footprint_deviation_yaw_);
                        yfc = y + ego_vehicle->footprintcircle_deviation_*sin(yaw+3*M_PI/2-ego_vehicle->footprint_deviation_yaw_);
                    }

                    double footprintDist = std::sqrt(std::pow(xfc-obstacle.x_, 2)+std::pow(yfc-obstacle.y_, 2));

                    if(footprintDist<= obstacle.radius_ + footprint_clearance_radius)
                    {
                        std::cout << "Collide with Footprint Circle" << std::endl;
                        return true;
                    }
                }
            }
        }
    }

    return false;
}