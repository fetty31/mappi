#include "Controller.hpp"

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

#include <nav_msgs/Path.h>

#include <tf2/utils.h>

NanoMPPIcROS::NanoMPPIcROS()
{

}

~NanoMPPIcROS::NanoMPPIcROS()
{
    
}

void NanoMPPIcROS::initialize(std::string name, 
                                tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros)
{
    
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

}

PLUGINLIB_EXPORT_CLASS(NanoMPPIcROS, nav_core::BaseLocalPlanner)