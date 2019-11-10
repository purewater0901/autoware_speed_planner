#ifndef SPEED_PLANNER_COLLISION_CHECKER_H
#define SPEED_PLANNER_COLLISION_CHECKER_H

#include <cmath>
#include <vector>
#include <memory>
#include "speed_planner/obstacle.h"
#include "speed_planner/trajectory_loader.h"
#include "speed_planner/vehicle_info.h"

class CollisionChecker
{
public:
    CollisionChecker() = default;
    
    bool check(TrajectoryLoader& trajectory, 
               const std::vector<Obstacle>& obstacles,
               const std::unique_ptr<VehicleInfo>& ego_vehicle,
               std::pair<double, double>& result);
};

#endif