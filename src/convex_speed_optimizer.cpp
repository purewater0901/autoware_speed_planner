#include "speed_planner/convex_speed_optimizer.h"

ConvexSpeedOptimizer::ConvexSpeedOptimizer
                       (const double previewDistance, 
                        const double ds, 
                        const double mass,
                        const double mu,
                        std::array<double, 5>& weight)
: previewDistance_(previewDistance), 
  ds_(ds), 
  mass_(mass), 
  mu_(mu), 
  gravity_(9.83), 
  epsilon_(1e-6),
  weight_(weight)
{
    if(ds_<1e-10)
        ds_ = 0.1;
}

bool ConvexSpeedOptimizer::calcOptimizedSpeed(const Trajectory& trajectory,
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
                                              const std::unique_ptr<CollisionInfo>& collision_info,
                                              const double safeTime)
{
    int N = trajectory.x_.size();
    int variableSize = 8*N-1+2*(N-1);

    assert(result_speed.size() == N);
    assert(result_acceleration.size() == N);

    try
    {
        /* Create Environment */
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set(GRB_IntParam_OutputFlag, 0);

        //Create Variables
        std::vector<GRBVar> variables;
        variables.resize(variableSize);

        variables[0] = model.addVar(Vd[0]*Vd[0], Vd[0]*Vd[0], 0.0, GRB_CONTINUOUS, "b"+std::to_string(0));
        for(int i=1; i<N; ++i)
            variables[i] = model.addVar(0.0, (Vr[i])*(Vr[i]), 0.0, GRB_CONTINUOUS, "b"+std::to_string(i));

        variables[N] = model.addVar(a0, a0, 0.0, GRB_CONTINUOUS, "a0");
        for(int i=N+1; i<2*N-1; ++i)
            variables[i] = model.addVar(-Arlon[i-N], Arlon[i-N], 0.0, GRB_CONTINUOUS, "a"+std::to_string(i-N));
        variables[2*N-1] = model.addVar(0.0, 0.0, 0.0, GRB_CONTINUOUS, "a"+std::to_string(N-1));

        for(int i=2*N; i<4*N; ++i)
            variables[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "slack"+std::to_string(i-2*N));

        for(int i=4*N; i<6*N; ++i)
            variables[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "u"+std::to_string(i-4*N));

        for(int i=6*N; i<7*N; ++i)
            variables[i] = model.addVar(epsilon_, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "cv"+std::to_string(i-6*N));

        for(int i=7*N; i<8*N-1; ++i)
            variables[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d"+std::to_string(i-7*N));

        for(int i=8*N-1; i<8*N-1+(N-1)*2; ++i)
            variables[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "spe"+std::to_string(i-(8*N-1)));

        //set objective
        GRBLinExpr  Jt = 0.0;
        GRBQuadExpr Js = 0.0;
        GRBQuadExpr Jv = 0.0;
        GRBQuadExpr JLonSlack = 0.0;
        GRBQuadExpr JLatSlack = 0.0;
        for(size_t i=0; i<N-1; ++i)
        {
            Jt += 2*ds_*variables[i+7*N];
            Js += (variables[i+N+1]-variables[i+N])*(variables[i+N+1]-variables[i+N])/ds_;
            Jv += (variables[i] - Vd[i]*Vd[i])*(variables[i]-Vd[i]*Vd[i])*ds_;
            JLonSlack += variables[i+2*N] * variables[i+2*N];
            JLatSlack += variables[i+3*N] * variables[i+3*N];
        }
        Jv += (variables[N-1] - Vd[N-1]*Vd[N-1])*(variables[N-1] - Vd[N-1]*Vd[N-1])*ds_;
        JLonSlack += variables[3*N-1] * variables[3*N-1];
        JLatSlack += variables[4*N-1] * variables[4*N-1];

        //model.setObjective(weight_[0]*Jt+weight_[1]*Js+weight_[2]*Jv+weight_[3]*JLonSlack+weight_[4]*JLatSlack, GRB_MINIMIZE);
        model.setObjective(weight_[1]*Js+weight_[2]*Jv+weight_[3]*JLonSlack+weight_[4]*JLatSlack, GRB_MINIMIZE);

        /* constraint1 speed and acceleration constraints */
        for(int i=0; i<N-1; ++i)
            model.addConstr((variables[i+1]-variables[i])/ds_ - 2*variables[i+N] == 0, "c"+std::to_string(i));

        // constraints2 longitudinal and lateral acceleration
        // longitudinal comfort constraints
        for(int i=0; i<N; ++i)
        {
            model.addConstr(Aclon[i]+variables[i+2*N] >= variables[i+N], "absconstr"+std::to_string(i));
            model.addConstr(-(Aclon[i]+variables[i+2*N]) <= variables[i+N], "absconstr"+std::to_string(i+N));
        }

        // lateral comfort constraints
        for(int i=0; i<N; ++i)
        {
            model.addConstr(mass_*(Aclat[i]+variables[i+3*N]) >= variables[i+5*N], "c"+std::to_string(i+N));
            model.addConstr(-mass_*(Aclat[i]+variables[i+3*N]) <= variables[i+5*N], "c"+std::to_string(i+2*N));
        }

        // constraints3 friction circle
        for(int i=0; i<N; ++i)
        {
            model.addQConstr(variables[i+4*N]*variables[i+4*N]+variables[i+5*N]*variables[i+5*N]<=(mu_*mass_*gravity_)*(mu_*mass_*gravity_),
                            "qcfc"+std::to_string(i));
            model.addConstr(variables[i+4*N]<=mass_*Arlon[i], "c"+std::to_string(i+3*N));
        }

        // constraint4  dynamics constraints
        for(int i=2; i<N-1; ++i)
        {
            double rx1 = (trajectory.x_[i+1]-trajectory.x_[i])/ds_;
            double rx2 = (trajectory.x_[i-2]-trajectory.x_[i-1]-trajectory.x_[i]+trajectory.x_[i+1])/(2*ds_*ds_);
            model.addConstr(variables[i+4*N]*cos(trajectory.yaw_[i]) - variables[i+5*N]*sin(trajectory.yaw_[i]) ==
                           (rx2 * variables[i] + rx1 * variables[i+N]), "c"+std::to_string(i+4*N));

            double ry1 = (trajectory.y_[i+1]-trajectory.y_[i])/ds_;
            double ry2 = (trajectory.y_[i-2]-trajectory.y_[i-1]-trajectory.y_[i]+trajectory.y_[i+1])/(2*ds_*ds_);
            model.addConstr(variables[i+4*N]*sin(trajectory.yaw_[i]) + variables[i+5*N]*cos(trajectory.yaw_[i]) ==
                           (ry2 * variables[i] + ry1 * variables[i+N]), "c"+std::to_string(i+5*N-3));
        }

        /* constraints5 time window constraints */
        if(is_collide)
        {
            if(collision_info->getType() == Obstacle::TYPE::DYNAMIC)
            {
                for(int i=0; i<N-1; ++i)
                {
                    model.addConstr(variables[i+(8*N-1)]==variables[i+6*N+1]+variables[i+6*N]-variables[i+7*N]+epsilon_, "se1"+std::to_string(i));
                    model.addConstr(variables[i+(9*N-2)]==variables[i+6*N+1]+variables[i+6*N]+variables[i+7*N]+epsilon_, "se2"+std::to_string(i));
                    model.addQConstr(4+variables[i+(8*N-1)]*variables[i+(8*N-1)]<=variables[i+(9*N-2)]*variables[i+(9*N-2)], "qc_time_window"+std::to_string(i));
                }

                for(int i=0; i<N; ++i)
                {
                    model.addQConstr(4*variables[i+6*N]*variables[i+6*N]+(variables[i]-1)*(variables[i]-1)<=(variables[i]+1)*(variables[i]+1), "qc"+std::to_string(i));
                    //model.addQConstr(variables[i]>=variables[i+6*N]*variables[i+6*N], "qc"+std::to_string(i));
                }

                GRBLinExpr timeWindowLin=0.0;
                double collision_time = collision_info->getCollisionTime();
                double traversal_time = collision_info->getTraversalTime();
                for(int i=0; i<collision_info->getId(); ++i)
                    timeWindowLin += 2*ds_*variables[i+7*N];
                model.addConstr(timeWindowLin-collision_time-traversal_time<=safeTime-collision_time-traversal_time, "timeWindow1");
                model.addConstr(timeWindowLin-collision_time-traversal_time>=0.0, "timeWindow2");
                //model.addConstr(timeWindowLin<=collision_info->getTime(), "timeWindow1");
                //model.addConstr(timeWindowLin>=0.0, "timeWindow2");
            }
        }

        //Optimization step
        model.optimize();

        for(int i=0; i<N; ++i)
        {
            result_speed[i] = std::sqrt(variables[i].get(GRB_DoubleAttr_X));
            result_acceleration[i] = variables[i+N].get(GRB_DoubleAttr_X);
        }
    }
    catch(GRBException e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;

        return false;
    }
    catch(...)
    {
        std::cout << "Exception during optimization" << std::endl;
        
        return false;
    }

    return true;
}