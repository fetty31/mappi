#include "Controller.hpp"

#include <pluginlib/class_list_macros.h>
// #include <base_local_planner/goal_functions.h>

#include <nav_msgs/Path.h>

namespace nano_mppic {

MPPIcROS::MPPIcROS() : tf_(NULL), initialized_(false)
{

}

MPPIcROS::MPPIcROS(std::string name, 
                    tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros)
    : tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

MPPIcROS::~MPPIcROS()
{
    nano_mppic_.shutdown();
}

void MPPIcROS::initialize(std::string name, 
                            tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros)
{

    ROS_INFO("NANO_MPPIC:: Initializing...");

    // Set up private ROS handlers (tf, costmap, node)
    tf_ = tf;
    costmap_ros_ptr_ = shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros);

    ros::NodeHandle nh("~/" + name);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>( "/ona2/fast_limo/state", 1,
                    boost::bind( &MPPIcROS::odom_callback, this, _1 ));

    /*To-Do:
        - fill nano_mppic::config obj with ROS params
    */

    config::MPPIc config;

    int batch_size, num_iters, time_steps, num_retry, offset;
    nh.param<int>("GeneralSettings/batch_size",  batch_size,  1000);
    nh.param<int>("GeneralSettings/num_iters",   num_iters,   4);
    nh.param<int>("GeneralSettings/time_steps",  time_steps,  40);
    nh.param<int>("GeneralSettings/num_retry",   num_retry,   2);
    nh.param<int>("GeneralSettings/offset",      offset,      1);

    config.settings.num_iters  = static_cast<unsigned int>(num_iters);
    config.settings.num_retry   = static_cast<unsigned int>(num_retry);
    config.settings.offset      = static_cast<unsigned int>(offset);
    
    config.noise.batch_size = static_cast<unsigned int>(batch_size);
    config.noise.time_steps = static_cast<unsigned int>(time_steps);

    nh.param<float>("GeneralSettings/model_dt",     config.model_dt,    0.01f);
    nh.param<float>("GeneralSettings/temperature",  config.temperature, 1.0f);
    nh.param<float>("GeneralSettings/gamma",        config.gamma,       1.0f);

    nh.param<float>("Ackermann/min_radius",  config.ackermann.min_r,  3.0f);

    nh.param<std::string>("MotionModel", config.settings.motion_model, "Ackermann");

    nh.param<float>("NoiseGeneration/std_vx", config.noise.std_vx, 0.01f);
    nh.param<float>("NoiseGeneration/std_vy", config.noise.std_vy, 0.01f);
    nh.param<float>("NoiseGeneration/std_wz", config.noise.std_wz, 0.01f);

    nh.param<float>("Constraints/max_vx", config.bounds.max_vx, 2.0f);
    nh.param<float>("Constraints/min_vx", config.bounds.min_vx, -1.0f);
    nh.param<float>("Constraints/max_vy", config.bounds.max_vy, 1.0f);
    nh.param<float>("Constraints/min_vy", config.bounds.min_vy, -1.0f);
    nh.param<float>("Constraints/max_wz", config.bounds.max_wz, 0.7f);
    nh.param<float>("Constraints/min_wz", config.bounds.min_wz, -0.7f);

    config.print_out(); // print out config (debug)

    nano_mppic_.configure(config, costmap_ros_ptr_);

    // Set up Visualizer instance
    vis_ptr_ = std::make_unique<Visualizer>(&nano_mppic_, &nh);

    initialized_ = true;

    ROS_INFO("NANO_MPPIC:: Finished initializing");
}

bool MPPIcROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{

    ROS_INFO("NANO_MPPIC:: Start computing new Velocity commands");

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
    
    ROS_INFO("NANO_MPPIC:: accessing nano_mppic controller for optimal cmd_vel");

    objects::Control cmd = nano_mppic_.getControl(current_odom_, global_plan_);
    cmd_vel.linear.x  = cmd.vx;
    cmd_vel.linear.y  = cmd.vy;
    cmd_vel.angular.z = cmd.wz;

    ROS_INFO("NANO_MPPIC:: Velocity commands computed!");

    return true;
}

bool MPPIcROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    ROS_INFO("NANO_MPPIC:: setting new global plan");
    ros_utils::ros2mppic(global_plan, global_plan_);
    return true;
}

bool MPPIcROS::isGoalReached()
{
    if (not is_initialized()) {
        ROS_ERROR("NANO_MPPIC: this planner/controller has not been initialized, please call initialize() before using this planner");
        return false;
    }

    ROS_INFO("NANO_MPPIC:: checking if Goal is reached");

    // if()
    // {
    //     ROS_INFO("NANO_MPPIC: Goal reached!");
    //     return true;
    // }
    // else
        return false;
}

void MPPIcROS::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros_utils::ros2mppic(*msg, current_odom_);
    ROS_INFO_ONCE("NANO_MPPIC::odom_callback started");
}

bool MPPIcROS::is_initialized()
{
    return initialized_;
}

} // namespace nano_mppic

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nano_mppic::MPPIcROS, nav_core::BaseLocalPlanner)