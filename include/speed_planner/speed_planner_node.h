#ifndef SPEED_PLANNER_NODE_ROS_H
#define SPEED_PLANNER_NODE_ROS_H

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <random>
#include <chrono>
#include <cstring>
#include <memory>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include "speed_planner/trajectory_loader.h"
#include "speed_planner/convex_speed_optimizer.h"
#include "speed_planner/obstacle.h"
#include "speed_planner/vehicle_info.h"
#include "speed_planner/collision_checker.h"
#include "speed_planner/trajectory.h"
#include "speed_planner/utils.h"

class SpeedPlannerNode
{
    public:
        SpeedPlannerNode();
        ~SpeedPlannerNode() = default;

    private:
        ros::NodeHandle nh_, private_nh_;
        ros::Publisher optimized_waypoints_pub_;
        ros::Publisher result_velocity_pub_;
        ros::Publisher optimized_waypoints_debug_;
        ros::Publisher curvature_pub_;
        ros::Publisher desired_velocity_pub_;
        ros::Subscriber current_status_sub_;
        ros::Subscriber current_pose_sub_;
        ros::Subscriber current_velocity_sub_;
        ros::Subscriber final_waypoints_sub_;
        ros::Subscriber objects_sub_;

        ros::Timer timer_;

        std::unique_ptr<tf2_ros::Buffer> tf2_buffer_ptr_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listner_ptr_;
  
        std::unique_ptr<autoware_msgs::Lane> in_lane_ptr_;
        std::unique_ptr<autoware_msgs::VehicleStatus> in_status_ptr_;
        std::unique_ptr<geometry_msgs::PoseStamped> in_pose_ptr_;
        std::unique_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
        std::unique_ptr<geometry_msgs::PoseStamped> in_nav_goal_ptr_;
        std::unique_ptr<autoware_msgs::DetectedObjectArray> in_objects_ptr_;
  
        std::unique_ptr<ConvexSpeedOptimizer> speedOptimizer_;
        std::unique_ptr<VehicleInfo> ego_vehicle_ptr_;
        std::unique_ptr<CollisionChecker> collision_checker_ptr_;
        std::unique_ptr<Trajectory> previous_trajectory_;

        void waypointsCallback(const autoware_msgs::Lane& msg);
        void objectsCallback(const autoware_msgs::DetectedObjectArray& msg);
        void currentStatusCallback(const autoware_msgs::VehicleStatus& msg);
        void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
        void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
        void timerCallback(const ros::TimerEvent &e);

        double curvatureWeight_;
        double decayFactor_;
        double previousVelocity_;
        double timer_callback_dt_;
        double lateral_g_;
        int skip_size_;
        int smooth_size_;
};

#endif