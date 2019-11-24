#include "speed_planner/collision_checker.h"

bool CollisionChecker::check(const Trajectory& trajectory, 
                             const std::vector<std::shared_ptr<Obstacle>>& obstacles,
                             const std::unique_ptr<VehicleInfo>& ego_vehicle,
                             std::pair<int, double>& result)
{
    if(obstacles.empty())
        return false;

    bool is_collide = false;
    for(int i=0; i<obstacles.size(); ++i)
    {
        Obstacle::TYPE obstacle_type = obstacles[i]->getType(); 
        if(obstacle_type == Obstacle::TYPE::STATIC)
            is_collide = static_obstacle_check(trajectory, obstacles[i], ego_vehicle, result);
        else if(obstacle_type == Obstacle::TYPE::DYNAMIC)
            is_collide = dynamic_obstacle_check(trajectory, obstacles[i], ego_vehicle, result);
    }

    return is_collide;
}

int CollisionChecker::collision_check(const Trajectory& trajectory,
                                      const std::unique_ptr<VehicleInfo>& ego_vehicle,
                                      const double obstacle_x,
                                      const double obstacle_y,
                                      const double obstacle_radius)
{
    int check_interval_size = 1.0/0.1;
    for(size_t i=0; i<trajectory.x_.size(); i+=check_interval_size)
    {
        //step1 check the largest radius around the vehicle
        double x = trajectory.x_[i];  //vehicle center's x coordinate 
        double y = trajectory.y_[i];  //vehicle center's y coordinate
        double yaw = trajectory.yaw_[i];
        double dist = std::sqrt(std::pow((x - obstacle_x), 2) + std::pow((y - obstacle_y), 2));
        double clearance_radius = ego_vehicle->circumcircle_radius_+ego_vehicle->safety_distance_; //largest_circle + safety_distance

        if(dist <= obstacle_radius+ clearance_radius)
        {
            //step2 check the middle circles
            double middle_clearance_radius = ego_vehicle->middlecircle_radius_ + ego_vehicle->safety_distance_;
            for(size_t mcircleId=0; mcircleId<2; ++mcircleId)
            {
                double xmc = x + ego_vehicle->middlecircle_deviation_*cos(yaw+mcircleId*M_PI);
                double ymc = y + ego_vehicle->middlecircle_deviation_*sin(yaw+mcircleId*M_PI);
                double middleDist = std::sqrt(std::pow(xmc-obstacle_x, 2)+std::pow(ymc-obstacle_y, 2));

                if(middleDist <= obstacle_radius + middle_clearance_radius)
                {
                    std::cout << "Collide with Middle Circle" << std::endl;
                    return i; //collision
                }
            }

            //step3 check the 4 footprint circles
            double footprint_clearance_radius = ego_vehicle->footprintcircle_radius_ + ego_vehicle->safety_distance_;
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

                double footprintDist = std::sqrt(std::pow(xfc-obstacle_x, 2)+std::pow(yfc-obstacle_y, 2));

                if(footprintDist<= obstacle_radius + footprint_clearance_radius)
                {
                    std::cout << "Collide with Footprint Circle" << std::endl;
                    return i;
                }
            }
        }
    }

    return -1;
}

bool CollisionChecker::static_obstacle_check(const Trajectory& trajectory,
                                             const std::shared_ptr<Obstacle>& obstacle,
                                             const std::unique_ptr<VehicleInfo>& ego_vehicle,
                                             std::pair<int, double>& result)
{
    std::cout << "This is Static Obstacle" << std::endl;
    double obstacle_x = obstacle->getPosition().begin()->second.first;
    double obstacle_y = obstacle->getPosition().begin()->second.second;
    double obstacle_radius = obstacle->getRadius();

    int collision_id = collision_check(trajectory, ego_vehicle, obstacle_x, obstacle_y, obstacle_radius);

    if(collision_id>=0)
    {
        result.first = collision_id; //collision position id
        result.second = 0.0;         //collision time
        return true;
    }

    return false;
}

bool CollisionChecker::dynamic_obstacle_check(const Trajectory& trajectory,
                                              const std::shared_ptr<Obstacle>& obstacle,
                                              const std::unique_ptr<VehicleInfo>& ego_vehicle,
                                              std::pair<int, double>& result)
{
    std::cout << "This is Dynamic Obstacle" << std::endl;
    double obstacle_radius = obstacle->getRadius();

    std::vector<std::pair<double, std::pair<double, double>>> obstacle_trajectory = obstacle->getPosition();
    for(int i=0 ;i<obstacle_trajectory.size(); ++i)
    {
        double obstacle_x = obstacle_trajectory[i].second.first;
        double obstacle_y = obstacle_trajectory[i].second.second;

        int collision_id = collision_check(trajectory, ego_vehicle, obstacle_x, obstacle_y, obstacle_radius);
        if(collision_id>=0)
        {
            result.first = collision_id;                  // collision position id
            result.second = obstacle_trajectory[i].first; // collision time

            return true;
        }
    }

    return false;
}