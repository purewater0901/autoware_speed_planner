#include "speed_planner/speed_planner_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_planner");
    SpeedPlannerNode node;
    ros::spin();

    return 0;
}

SpeedPlannerNode::SpeedPlannerNode() : nh_(), private_nh_("~"), isInitialize_(false), timer_callback_dt_(0.1)
{
    double mass;
    double mu;
    double ds;
    double previewDistance;
    std::array<double, 5> weight;
    private_nh_.param<double>("mass", mass, 1500.0);
    private_nh_.param<double>("mu", mu, 0.8);
    private_nh_.param<double>("ds", ds, 0.1);
    private_nh_.param<double>("preview_distance", previewDistance, 20);
    private_nh_.param<double>("curvature_weight", curvatureWeight_, 20);
    private_nh_.param<double>("decay_factor", decayFactor_, 0.8);
    private_nh_.param<double>("time_weight", weight[0], 0.0);
    private_nh_.param<double>("smooth_weight", weight[1], 15.0);
    private_nh_.param<double>("velocity_weight", weight[2], 0.001);
    private_nh_.param<double>("longitudinal_slack_weight", weight[3], 1.0);
    private_nh_.param<double>("lateral_slack_weight", weight[4], 10.0);

    speedOptimizer_.reset(new ConvexSpeedOptimizer(previewDistance, ds, mass, mu, weight));

    optimized_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);
    optimized_waypoints_debug_ = nh_.advertise<geometry_msgs::Twist>("optimized_speed_debug", 1, true);
    final_waypoints_sub_ = nh_.subscribe("safety_waypoints", 1, &SpeedPlannerNode::waypointsCallback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &SpeedPlannerNode::currentPoseCallback, this);
    current_status_sub_ = nh_.subscribe("/vehicle_status", 1, &SpeedPlannerNode::currentStatusCallback, this);
    current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &SpeedPlannerNode::currentVelocityCallback, this);
    nav_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SpeedPlannerNode::navGoalCallback, this);
    objects_sub_ = nh_.subscribe("/detection/fake_perception/objects", 1, &SpeedPlannerNode::objectsCallback, this);
    timer_ = nh_.createTimer(ros::Duration(timer_callback_dt_), &SpeedPlannerNode::timerCallback, this);
}

void SpeedPlannerNode::waypointsCallback(const autoware_msgs::Lane& msg)
{
  in_lane_ptr_.reset(new autoware_msgs::Lane(msg));
}

void SpeedPlannerNode::currentPoseCallback(const geometry_msgs::PoseStamped & msg)
{
  in_pose_ptr_.reset(new geometry_msgs::PoseStamped(msg));
}

void SpeedPlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  
  in_twist_ptr_.reset(new geometry_msgs::TwistStamped(msg));
}

void SpeedPlannerNode::currentStatusCallback(const autoware_msgs::VehicleStatus& msg)
{
  in_status_ptr_.reset(new autoware_msgs::VehicleStatus(msg));
}

void SpeedPlannerNode::navGoalCallback(const geometry_msgs::PoseStamped& msg)
{
  std::string target_frame = "map";
  std::string source_frame = "world";
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf2_buffer_ptr_->lookupTransform(target_frame, source_frame, ros::Time(0));
    geometry_msgs::PoseStamped msg_in_map;
    tf2::doTransform(msg, msg_in_map, transform);
    in_nav_goal_ptr_.reset(new geometry_msgs::PoseStamped(msg_in_map));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void SpeedPlannerNode::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if(in_lane_ptr_)
  {
    if(msg.objects.size() == 0)
    {
      std::cerr << "ssize of objects is 0" << std::endl;
      return;
    }
    geometry_msgs::TransformStamped objects2map_tf;
    try
    {
        objects2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_lane_ptr_->header.frame_id, 
          /*src*/ msg.header.frame_id,
          ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    in_objects_ptr_.reset(new autoware_msgs::DetectedObjectArray(msg));
    in_objects_ptr_->header.frame_id = in_lane_ptr_->header.frame_id;
    for(auto& object: in_objects_ptr_->objects)
    {
      object.header.frame_id = in_lane_ptr_->header.frame_id;
      geometry_msgs::PoseStamped current_object_pose;
      current_object_pose.header = object.header;
      current_object_pose.pose = object.pose;
      geometry_msgs::PoseStamped transformed_pose;
      tf2::doTransform(current_object_pose, transformed_pose, objects2map_tf);
      object.pose = transformed_pose.pose;
    }
  }
}

void SpeedPlannerNode::timerCallback(const ros::TimerEvent& e)
{
    if(in_lane_ptr_ && in_twist_ptr_)
    {
        int waypointSize = in_lane_ptr_->waypoints.size();
        std::vector<double> x(waypointSize, 0.0);
        std::vector<double> y(waypointSize, 0.0);

        for(size_t id=0; id<waypointSize; ++id)
        {
            x[id] = in_lane_ptr_->waypoints[id].pose.pose.position.x;
            y[id] = in_lane_ptr_->waypoints[id].pose.pose.position.y;
        }

        //1. create trajectory
        double previewDistance = speedOptimizer_->previewDistance_;
        int skip_size = 5;
        int smooth_size = 50;
        TrajectoryLoader trajectory(x, y, speedOptimizer_->ds_, previewDistance, skip_size, smooth_size);

        //2. Create Speed Constraints and Acceleration Constraints
        int N = trajectory.size();
        std::vector<double> Vr(N, 0.0);     //restricted speed array
        std::vector<double> Vd(N, 0.0);     //desired speed array
        std::vector<double> Arlon(N, 0.0);  //acceleration longitudinal restriction
        std::vector<double> Arlat(N, 0.0);  //acceleration lateral restriction
        std::vector<double> Aclon(N, 0.0);  //comfort longitudinal acceleration restriction
        std::vector<double> Aclat(N, 0.0);  //comfort lateral acceleration restriction

        trajectory.curvature_[0] = trajectory.curvature_[1];
        for(size_t i=0; i<Vr.size(); ++i)
        {
            Vr[i] = 5.0;
            //Vd[i] = Vr[i]/(1+curvatureWeight_*std::fabs(trajectory.curvature_[i]));

            double tmpSum = 0.0;
            for(size_t j=i; j<i+10; ++j)
            {
                if(j<Vr.size())
                  tmpSum += curvatureWeight_*std::pow(decayFactor_, (j-i))*std::fabs(trajectory.curvature_[j]);
                else
                  break;
            }
            tmpSum+=1.0;
            Vd[i] = Vr[i]/tmpSum;
        }

        double mu = speedOptimizer_->mu_;
        for(size_t i=0; i<N; ++i)
        {
            Arlon[i] = 0.5*mu*9.83;
            Arlat[i] = 0.5*mu*9.83;
            Aclon[i] = 0.4*mu*9.83;
            Aclat[i] = 0.4*mu*9.83;
        }

        //3. initial speed and initial acceleration
        double v0 = in_twist_ptr_->twist.linear.x;
        std::cout << "Current Velocity: " << v0 << std::endl;
        double a0 = 0.0;
        if(!isInitialize_)
        {
          isInitialize_ = true;
          previousVelocity_ = v0;
        }
        else
        {
          a0 = (v0-previousVelocity_)/timer_callback_dt_;
          previousVelocity_ = v0;
        }

        //4. dyanmic 
        double collisionTime = 0.0;
        double collisionDistance = 0.0;
        double safeTime = 0.0;

        //Output the information
        std::cout << "==================== Size: " << N  << "======================"<< std::endl;

        //////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////Calculate Optimized Speed////////////////////////////////
        std::vector<double> result(N, 0.0);
        if(!speedOptimizer_->calcOptimizedSpeed(trajectory, result, Vr, Vd, Arlon, Arlat, Aclon, Aclat, v0, a0, collisionTime, collisionDistance, safeTime))
            return;

        // Show result
        /*
        for(int i=0; i<result.size(); ++i)
          std::cout << speedOptimizer_->ds_*i << ": " << result[i] << std::endl;
          */

        //5. set result
        autoware_msgs::Lane speedOptimizedLane;
        speedOptimizedLane.lane_id = in_lane_ptr_->lane_id;
        speedOptimizedLane.lane_index = in_lane_ptr_->lane_index;
        speedOptimizedLane.is_blocked = in_lane_ptr_->is_blocked;
        speedOptimizedLane.increment = in_lane_ptr_->increment;
        speedOptimizedLane.header = in_lane_ptr_->header;
        speedOptimizedLane.cost = in_lane_ptr_->cost;
        speedOptimizedLane.closest_object_distance = in_lane_ptr_->closest_object_distance;
        speedOptimizedLane.closest_object_velocity = in_lane_ptr_->closest_object_velocity;
        speedOptimizedLane.waypoints.reserve(result.size());

        int scale = 1.0/speedOptimizer_->ds_;
        for(int i=10; i<N; i++)
        {
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = trajectory.x_[i];
            waypoint.pose.pose.position.y = trajectory.y_[i];
            waypoint.pose.pose.position.z = in_lane_ptr_->waypoints[0].pose.pose.position.z;
            waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.yaw_[i]);
            waypoint.pose.header = in_lane_ptr_->header;
            waypoint.twist.header=in_lane_ptr_->header;
            if(result[i]+0.01>5.0)
              waypoint.twist.twist.linear.x = 4.9;
            else
              waypoint.twist.twist.linear.x = (result[i]);
            speedOptimizedLane.waypoints.push_back(waypoint);
        }

        for(int i=0; i<result.size(); ++i)
        {
          std::cout << Vd[i] << ": " << result[i] << std::endl;
        }

        optimized_waypoints_pub_.publish(speedOptimizedLane);
    }

}
