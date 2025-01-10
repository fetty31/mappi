#include "Controller.hpp"

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

#include <nav_msgs/Path.h>

#include <tf2/utils.h>

NanoMPPIcROS::NanoMPPIcROS() : initialized_(false)
{

}

NanoMPPIcROS::~NanoMPPIcROS()
{
    nano_mppic_.shutdown();
}

void NanoMPPIcROS::initialize(std::string name, 
                                tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros)
{

    // Set up private ROS handlers (tf, costmap, node)
    tf_ = tf;
    costmap_ros_ptr_ = nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros);

    ros::NodeHandle nh("~/" + name);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>( "/fast_limo/state", 1,
                    boost::bind( &NanoMPPIcROS::odom_callback, this, _1 ));

    /*To-Do:
        - fill nano_mppic::config obj with ROS params
    */
    nano_mppic::config::MPPIc config;
    config.settings.num_iters = 4;
    config.settings.num_retry = 2;
    config.settings.offset = 1;
    config.settings.motion_model = "Ackermann";

    config.ackermann.min_r = 3.0;

    config.noise.batch_size = 1000;
    config.noise.time_steps = 40;
    config.noise.std_vx = 0.01;
    config.noise.std_vy = 0.01;
    config.noise.std_wz = 0.01;

    config.bounds.max_vx = 2.0;
    config.bounds.min_vx = -1.0;
    config.bounds.max_vy = 1.0;
    config.bounds.min_vy = -1.0;
    config.bounds.max_wz = 0.7;
    config.bounds.min_wz = 0.7;

    config.model_dt = 0.05;
    config.temperature = 1.0;
    config.gamma = 1.0;

    nano_mppic_.configure(config, costmap_ros_ptr_);

    initialized_ = true;
}

bool NanoMPPIcROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(not is_initialized()) return false;

    static geometry_msgs::PoseStamped current_pose;
    if(not costmap_ros_ptr_->getRobotPose(current_pose)){
        ROS_ERROR("NANO_MPPIC could not get robot pose!");
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }
    // nano_mppic::objects::Odometry2d odom;
    // ros_utils::ros2mppic(current_pose, odom);

    nano_mppic::objects::Control cmd = nano_mppic_.getControl(current_odom_, global_plan_);
    cmd_vel.linear.x  = cmd.vx;
    cmd_vel.linear.y  = cmd.vy;
    cmd_vel.angular.z = cmd.wz;
    return true;
}

bool NanoMPPIcROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    ros_utils::ros2mppic(global_plan, global_plan_);
    return true;
}

bool NanoMPPIcROS::isGoalReached()
{
    if (not is_initialized()) {
        ROS_ERROR("NANO_MPPIC: this planner/controller has not been initialized, please call initialize() before using this planner");
        return false;
    }

    // if()
    // {
    //     ROS_INFO("NANO_MPPIC: Goal reached!");
    //     return true;
    // }
    // else
        return false;
}

void NanoMPPIcROS::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros_utils::ros2mppic(*msg, current_odom_);
}

bool NanoMPPIcROS::is_initialized()
{
    return initialized_;
}

PLUGINLIB_EXPORT_CLASS(NanoMPPIcROS, nav_core::BaseLocalPlanner)