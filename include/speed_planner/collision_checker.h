#ifndef SPEED_PLANNER_COLLISION_CHECKER_H
#define SPEED_PLANNER_COLLISION_CHECKER_H

#include <cmath>
#include <vector>
#include <memory>
#include "speed_planner/obstacle.h"
#include "speed_planner/trajectory.h"
#include "speed_planner/vehicle_info.h"

class CollisionInfo
{
public:
    CollisionInfo(const Obstacle::TYPE type, int collision_position_id, double collision_time)
                 : type_(type), collision_position_id_(collision_position_id), collision_time_(collision_time), traversal_time_(10.0)
                 {}

    int getId() { return collision_position_id_; }
    double getCollisionTime() { return collision_time_; }
    double getTraversalTime() { return traversal_time_; }
    Obstacle::TYPE getType() { return type_; }

    Obstacle::TYPE type_;
    int collision_position_id_;
    double collision_time_;
    double traversal_time_;
};

class CollisionChecker
{
public:
    CollisionChecker() = default;
    
    bool check(const Trajectory& trajectory, 
               const std::vector<std::shared_ptr<Obstacle>>& obstacles,
               const std::unique_ptr<VehicleInfo>& ego_vehicle,
               std::unique_ptr<CollisionInfo>& result);

private:
    bool static_obstacle_check(const Trajectory& trajectory,
                               const std::shared_ptr<Obstacle>& obstacle,
                               const std::unique_ptr<VehicleInfo>& ego_vehicle,
                               std::unique_ptr<CollisionInfo>& result);

    bool dynamic_obstacle_check(const Trajectory& trajectory,
                               const std::shared_ptr<Obstacle>& obstacle,
                               const std::unique_ptr<VehicleInfo>& ego_vehicle,
                               std::unique_ptr<CollisionInfo>& result);

    int collision_check(const Trajectory& trajectory,
                        const std::unique_ptr<VehicleInfo>& ego_vehicle,
                        const double obstacle_x,
                        const double obstacle_y,
                        const double obstacle_radius);
};

#endif