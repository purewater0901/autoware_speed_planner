#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cassert>
#include "speed_planner/trajectory.h"
#include "speed_planner/collision_checker.h"
#include "gurobi_c++.h"

class ConvexSpeedOptimizer
{
    public:
        ConvexSpeedOptimizer(const double previewDistance, 
                        const double ds, 
                        const double mass,
                        const double mu,
                        std::array<double, 5>& weight);

        bool calcOptimizedSpeed(const Trajectory& trajectory,
                                std::vector<double>& result_speed, 
                                std::vector<double>& result_acceleration, 
                                const std::vector<double>& Vr,
                                const std::vector<double>& Vd,
                                const std::vector<double>& Arlon,
                                const std::vector<double>& Arlat,
                                const std::vector<double>& Aclon,
                                const std::vector<double>& Aclat,
                                const double a0,
                                const bool is_collide,
                                const std::unique_ptr<CollisionInfo>& collisioin_info,
                                const double safeTime);

        double epsilon_;
        double gravity_;
        double mass_;
        double mu_;

        double previewDistance_;
        double ds_;
        std::array<double, 5> weight_;
};