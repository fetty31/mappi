#include "Controller.hpp"

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

#include <nav_msgs/Path.h>

#include <tf2/utils.h>

NanoMPPIcROS::NanoMPPIcROS() : initialized_(false)
{

}

~NanoMPPIcROS::NanoMPPIcROS()
{
    
}

void NanoMPPIcROS::initialize(std::string name, 
                                tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle private_nh("~/" + name);
    tf_ = tf;

    costmap_ros_ = costmap_ros;

    /* To-Do:
        - fill config::Predictor obj
        - initialize nano_mppic_ obj
            . probably must change std::shared_ptr into boost::shared_ptr
            . make macro for changing std/boost 
    */
    // nano_mppic_.configure()

    initialized_ = true;
}

void NanoMPPIcROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{

}

bool NanoMPPIcROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{

}

bool NanoMPPIcROS::isGoalReached()
{

}

bool NanoMPPIcROS::is_initialized()
{
 return initialized_;
}

PLUGINLIB_EXPORT_CLASS(NanoMPPIcROS, nav_core::BaseLocalPlanner)