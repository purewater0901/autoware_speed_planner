#include "speed_planner/speed_planner_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_planner");
    SpeedPlannerNode node;
    ros::spin();

    return 0;
}

SpeedPlannerNode::SpeedPlannerNode() : nh_(), private_nh_("~"), timer_callback_dt_(0.1), previous_trajectory_()
{
    double mass;
    double mu;
    double ds;
    double previewDistance;
    double vehicle_length;
    double vehicle_width;
    double vehicle_wheel_base;
    double vehicle_safety_distance;
    std::array<double, 5> weight{0};

    private_nh_.param<double>("mass", mass, 1500.0);
    private_nh_.param<double>("mu", mu, 0.8);
    private_nh_.param<double>("ds", ds, 0.1);
    private_nh_.param<double>("preview_distance", previewDistance, 20.0);
    private_nh_.param<double>("curvature_weight", curvatureWeight_, 20.0);
    private_nh_.param<double>("decay_factor", decayFactor_, 0.8);
    private_nh_.param<double>("time_weight", weight[0], 0.0);
    private_nh_.param<double>("smooth_weight", weight[1], 15.0);
    private_nh_.param<double>("velocity_weight", weight[2], 0.001);
    private_nh_.param<double>("longitudinal_slack_weight", weight[3], 1.0);
    private_nh_.param<double>("lateral_slack_weight", weight[4], 10.0);
    private_nh_.param<double>("lateral_g", lateral_g_, 0.4);
    private_nh_.param<int>("skip_size", skip_size_, 10);
    private_nh_.param<int>("smooth_size", smooth_size_, 50);
    private_nh_.param<double>("vehicle_length", vehicle_length, 5.0);
    private_nh_.param<double>("vehicle_width", vehicle_width, 1.895);
    private_nh_.param<double>("vehicle_wheel_base", vehicle_wheel_base, 2.790);
    private_nh_.param<double>("vehicle_safety_distance", vehicle_safety_distance, 0.1);

    speedOptimizer_.reset(new ConvexSpeedOptimizer(previewDistance, ds, mass, mu, weight));
    ego_vehicle_ptr_.reset(new VehicleInfo(vehicle_length, vehicle_width, vehicle_wheel_base,vehicle_safety_distance));
    collision_checker_ptr_.reset(new CollisionChecker());
    tf2_buffer_ptr_.reset(new tf2_ros::Buffer());
    tf2_listner_ptr_.reset(new tf2_ros::TransformListener(*tf2_buffer_ptr_));

    optimized_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);
    optimized_waypoints_debug_ = nh_.advertise<geometry_msgs::Twist>("optimized_speed_debug", 1, true);
    result_velocity_pub_ = nh_.advertise<std_msgs::Float32>("result_velocity", 1, true);
    desired_velocity_pub_ = nh_.advertise<std_msgs::Float32>("desired_velocity", 1, true);
    curvature_pub_ = nh_.advertise<std_msgs::Float32>("curvature", 1, true);

    final_waypoints_sub_ = nh_.subscribe("safety_waypoints", 1, &SpeedPlannerNode::waypointsCallback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &SpeedPlannerNode::currentPoseCallback, this);
    current_status_sub_ = nh_.subscribe("/vehicle_status", 1, &SpeedPlannerNode::currentStatusCallback, this);
    current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &SpeedPlannerNode::currentVelocityCallback, this);
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

void SpeedPlannerNode::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if(in_lane_ptr_)
  {
    if(msg.objects.size() == 0)
    {
      std::cerr << "size of objects is 0" << std::endl;
      return;
    }
    geometry_msgs::TransformStamped objects2map_tf;
    try
    {
        objects2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_lane_ptr_->header.frame_id,  //map
          /*src*/ msg.objects.begin()->header.frame_id,  //lidar
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
    if(in_lane_ptr_&& in_twist_ptr_ && ego_vehicle_ptr_ && in_pose_ptr_)
    {
        int waypointSize = in_lane_ptr_->waypoints.size();
        std::vector<double> waypoint_x(waypointSize, 0.0);
        std::vector<double> waypoint_y(waypointSize, 0.0);
        std::vector<double> waypoint_yaw(waypointSize, 0.0);
        std::vector<double> waypoint_curvature(waypointSize, 0.0);
        double current_x = in_pose_ptr_->pose.position.x;
        double current_y = in_pose_ptr_->pose.position.y;

        for(size_t id=0; id<waypointSize; ++id)
        {
            waypoint_x[id] = in_lane_ptr_->waypoints[id].pose.pose.position.x;
            waypoint_y[id] = in_lane_ptr_->waypoints[id].pose.pose.position.y;
            waypoint_yaw[id] = tf::getYaw(in_lane_ptr_->waypoints[id].pose.pose.orientation);
            waypoint_curvature[id] = in_lane_ptr_->waypoints[id].pose.pose.position.z;
        }
        
        //1. create trajectory
        //TrajectoryLoader trajectory(current_x, current_y, waypoint_x, waypoint_y, speedOptimizer_->ds_, speedOptimizer_->previewDistance_, skip_size_, smooth_size_);
        int nearest_waypoint_id = getNearestId(current_x, current_y, waypoint_x, waypoint_y);
        Trajectory trajectory(waypoint_x, waypoint_y, waypoint_yaw, waypoint_curvature, nearest_waypoint_id);

        //2. initial speed and initial acceleration
        //initial velocity
        int nearest_previous_point_id=0;
        double v0 = 0.0;
        double a0 = 0.0;

        if(previous_trajectory_==nullptr)
        {
          double v0 = in_twist_ptr_->twist.linear.x;
          previousVelocity_ = v0;
        }
        else
        {
          nearest_previous_point_id = getNearestId(current_x, current_y, previous_trajectory_->x_, previous_trajectory_->y_, 2);
          v0 = previous_trajectory_->velocity_[nearest_previous_point_id];
          a0 = previous_trajectory_->acceleration_[nearest_previous_point_id];
          previousVelocity_ = v0;

          ROS_INFO("Nearest id is %d", nearest_previous_point_id);
        }
        
        ROS_INFO("Value of v0: %f", v0);
        ROS_INFO("Value of a0: %f", a0);
        ROS_INFO("Current Velocity: %f", in_twist_ptr_->twist.linear.x);

        //3. Create Speed Constraints and Acceleration Constraints
        int N = trajectory.x_.size();
        std::vector<double> Vr(N, 0.0);     //restricted speed array
        std::vector<double> Vd(N, 0.0);     //desired speed array
        std::vector<double> Arlon(N, 0.0);  //acceleration longitudinal restriction
        std::vector<double> Arlat(N, 0.0);  //acceleration lateral restriction
        std::vector<double> Aclon(N, 0.0);  //comfort longitudinal acceleration restriction
        std::vector<double> Aclat(N, 0.0);  //comfort lateral acceleration restriction

        Vr[0] = 5.0;
        Vd[0] = v0;
        for(size_t i=1; i<Vr.size(); ++i)
        {
            Vr[i] = 5.0;
            Vd[i] = std::max(std::min(Vr[i]-0.5, std::sqrt(lateral_g_/(std::fabs(trajectory.curvature_[i]+1e-10)))), 1.0);
        }

        double mu = speedOptimizer_->mu_;
        for(size_t i=0; i<N; ++i)
        {
            Arlon[i] = 0.5*mu*9.83;
            Arlat[i] = 0.5*mu*9.83;
            Aclon[i] = 0.4*mu*9.83;
            Aclat[i] = 0.4*mu*9.83;
        }

        //4. dyanmic obstacles
        double safeTime = 10.0;
        std::pair<double, double> collision_time_distance_result; //predicted collision time and distance
        bool is_collide = false;

        std::vector<Obstacle> obstacles;
        if(in_objects_ptr_ && !in_objects_ptr_->objects.empty())
        {
          for(int i=0; i<in_objects_ptr_->objects.size(); ++i)
          {
            Obstacle tmp;
            tmp.x_ = in_objects_ptr_->objects[i].pose.position.x;
            tmp.y_ = in_objects_ptr_->objects[i].pose.position.y;
            tmp.radius_ = std::sqrt(std::pow(in_objects_ptr_->objects[i].dimensions.x, 2) + std::pow(in_objects_ptr_->objects[i].dimensions.y, 2));
            tmp.translational_velocity_ = in_objects_ptr_->objects[i].velocity.linear.x;
            obstacles.push_back(tmp);
          }

          is_collide = collision_checker_ptr_->check(trajectory, obstacles, ego_vehicle_ptr_, collision_time_distance_result);
        }

        if(is_collide)
          ROS_INFO("Collide");
        else
          ROS_INFO("Not Collide");

        double collisionTime=0.0;
        double collisionDistance = 0.0;

        //Output the information
        ROS_INFO("Size: %d", N);

        //////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////Calculate Optimized Speed////////////////////////////////
        std::vector<double> result_speed(N, 0.0);
        std::vector<double> result_acceleration(N, 0.0);
        bool is_result = speedOptimizer_->calcOptimizedSpeed(trajectory, result_speed, result_acceleration, Vr, Vd, Arlon, Arlat, Aclon, Aclat, a0, collisionTime, collisionDistance, safeTime);

        if(is_result)
        {
          ROS_INFO("Gurobi Success");
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
          speedOptimizedLane.waypoints.reserve(result_speed.size());

          for(int i=0; i<N; i++)
          {
            //result[i] = std::min(std::max(result[i], 0.5), 4.9);
            result_speed[i] = std::min(result_speed[i] ,4.9);
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = trajectory.x_[i];
            waypoint.pose.pose.position.y = trajectory.y_[i];
            waypoint.pose.pose.position.z = in_lane_ptr_->waypoints[0].pose.pose.position.z;
            waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.yaw_[i]);
            waypoint.pose.header = in_lane_ptr_->header;
            waypoint.twist.header=in_lane_ptr_->header;
            waypoint.twist.twist.linear.x = result_speed[i];

            speedOptimizedLane.waypoints.push_back(waypoint);
          }

          if(!result_speed.empty())
          {
            std_msgs::Float32 result_velocity;
            result_velocity.data = result_speed[0];
            result_velocity_pub_.publish(result_velocity);
            std_msgs::Float32 desired_velocity;
            desired_velocity.data = Vd[2];
            desired_velocity_pub_.publish(desired_velocity);
          }

          optimized_waypoints_pub_.publish(speedOptimizedLane);
          previous_trajectory_.reset(new Trajectory(waypoint_x, waypoint_y, waypoint_yaw, waypoint_curvature, result_speed, result_acceleration));

          std_msgs::Float32 curvature;
          curvature.data = trajectory.curvature_[0];
          curvature_pub_.publish(curvature);
        }
        else
        {
          ROS_INFO("Gurobi Fialed");
          std::cout << "Vr[0]: " << Vr[0] << std::endl;
          std::cout << "Vd[0]: " << Vd[0] << std::endl;
          std::cout << "Vr[1]: " << Vr[1] << std::endl;
          std::cout << "Vd[1]: " << Vd[1] << std::endl;
          std::cout << "Vr[2]: " << Vr[2] << std::endl;
          std::cout << "Vd[2]: " << Vd[2] << std::endl;
          std::cout << "V0: " << v0 << std::endl;
          std::cout << "a0: " << a0 << std::endl;
          //"if gurobi failed calculation"
          if(previous_trajectory_==nullptr)
            return;

          //5. set previous result
          autoware_msgs::Lane speedOptimizedLane;
          speedOptimizedLane.lane_id = in_lane_ptr_->lane_id;
          speedOptimizedLane.lane_index = in_lane_ptr_->lane_index;
          speedOptimizedLane.is_blocked = in_lane_ptr_->is_blocked;
          speedOptimizedLane.increment = in_lane_ptr_->increment;
          speedOptimizedLane.header = in_lane_ptr_->header;
          speedOptimizedLane.cost = in_lane_ptr_->cost;
          speedOptimizedLane.closest_object_distance = in_lane_ptr_->closest_object_distance;
          speedOptimizedLane.closest_object_velocity = in_lane_ptr_->closest_object_velocity;
          speedOptimizedLane.waypoints.reserve(trajectory.x_.size());

          for(int i=nearest_previous_point_id; i<N; i++)
          {
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = trajectory.x_[i-nearest_previous_point_id];
            waypoint.pose.pose.position.y = trajectory.y_[i-nearest_previous_point_id];
            waypoint.pose.pose.position.z = in_lane_ptr_->waypoints[0].pose.pose.position.z;
            waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.yaw_[i-nearest_previous_point_id]);
            waypoint.pose.header = in_lane_ptr_->header;
            waypoint.twist.header=in_lane_ptr_->header;
            waypoint.twist.twist.linear.x = previous_trajectory_->velocity_[i];

            speedOptimizedLane.waypoints.push_back(waypoint);
          }
        }

        ROS_INFO("=======================================================");
    }

}
