#pragma once

#include "mppic.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

namespace ros_utils {

void ros2mppic(const std::vector<geometry_msgs::PoseStamped>& in_path, 
                nano_mppic::objects::Path& out_path)
{
    out_path.reset(in_path.size());

    for (size_t i=0; i < in_path.size(); ++i) {
        out_path.x(i) = in_path[i].pose.position.x;
        out_path.y(i) = in_path[i].pose.position.y;
        out_path.yaw(i) = tf2::getYaw(in_path[i].pose.orientation);
    }
}

void ros2mppic(const geometry_msgs::PoseStamped& in_pose, 
                nano_mppic::objects::Odometry2d& out_pose)
{
    out_pose.x   = in_pose.pose.position.x;
    out_pose.y   = in_pose.pose.position.y;
    out_pose.yaw = tf2::getYaw(in_pose.pose.orientation);
    out_pose.stamp = static_cast<float>(in_pose.header.stamp.toSec());
}

void ros2mppic(const nav_msgs::Odometry& in_odom,
                nano_mppic::objects::Odometry2d& out_odom)
{
    out_odom.x   = in_odom.pose.pose.position.x;
    out_odom.y   = in_odom.pose.pose.position.y;
    out_odom.yaw = tf2::getYaw(in_odom.pose.pose.orientation);

    out_odom.vx = in_odom.twist.twist.linear.x;
    out_odom.vy = in_odom.twist.twist.linear.y;
    out_odom.wz = in_odom.twist.twist.angular.z;

    out_odom.stamp = static_cast<float>(in_odom.header.stamp.toSec());
}

} // namespace ros_utils