#pragma once

#include <iostream>
#include <vector>
#include <array>
#include "speed_planner/trajectory_loader.h"
#include "gurobi_c++.h"

class ConvexSpeedOptimizer
{
    public:
        ConvexSpeedOptimizer(const double previewDistance, 
                        const double ds, 
                        const double mass,
                        const double mu,
                        std::array<double, 5>& weight);

        bool calcOptimizedSpeed(TrajectoryLoader& trajectory,
                                std::vector<double>& resultSpeed, 
                                const std::vector<double>& Vr,
                                const std::vector<double>& Vd,
                                const std::vector<double>& Arlon,
                                const std::vector<double>& Arlat,
                                const std::vector<double>& Aclon,
                                const std::vector<double>& Aclat,
                                const double V0,
                                const double a0,
                                const double collisionTime,
                                const double collisionDistance,
                                const double safeTime);

        double epsilon_;
        double gravity_;
        double mass_;
        double mu_;

        double previewDistance_;
        double ds_;
        std::array<double, 5> weight_;
};