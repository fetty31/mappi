#pragma once

#include "nano_mppic/include/Predictor.hpp"

#include <xtensor/xadapt.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/tf2.h>
#include <tf2/utils.h>

namespace ros_utils {

inline nano_mppic::objects::Path ros2mppic(const std::vector<geometry_msgs::PoseStamped>& ros_path)
{

    auto path = nano_mppic::objects::Path{};
    path.reset(ros_path.size());

    for (auto pose : ros_path) {
        path.x(i) = pose.pose.position.x;
        path.y(i) = pose.pose.position.y;
        path.yaw(i) = tf2::getYaw(pose.pose.orientation);
    }

    // std::vector<float> x_values, y_values, yaw_values;
    // x_values.reserve(ros_path.size()); 
    // y_values.reserve(ros_path.size()); 
    // yaw_values.reserve(ros_path.size());

    // for(auto pose : ros_path){
    //     x_values.push_back(pose.pose.position.x);
    //     y_values.push_back(pose.pose.position.y);

    //     tf2::Quaternion q(
    //         pose.pose.orientation.x,
    //         pose.pose.orientation.y,
    //         pose.pose.orientation.z,
    //         pose.pose.orientation.w);
    //     tf2::Matrix3x3 m(q);
    //     static double roll, pitch, yaw;
    //     m.getRPY(roll, pitch, yaw);

    //     yaw_values.push_back(yaw);
    // }
    
    // nano_mppic::objects::Path path;
    // path.x   = xt::adapt(x_values);
    // path.y   = xt::adapt(y_values);
    // path.yaw = xt::adapt(yaw_values);

    return path;
}

inline nano_mppic::objects::Odometry2d ros2mppic(geometric_msgs::PoseStamped& pose_stamped)
{
    nano_mppic::objects::Odometry2d odom();
    odom.x   = pose_stamped.pose.position.x;
    odom.y   = pose_stamped.pose.position.y;
    odom.yaw = tf2::getYaw(pose_stamped.pose.orientation);
    odom.stamp = static_cast<float>(pose_stamped.header.stamp.toSec());

    return odom;
}

inline nano_mppic::objects::Odometry2d ros2mppic(nav_msgs::Odometry& ros_odom)
{
    nano_mppic::objects::Odometry2d odom();
    odom.x   = ros_odom.pose.pose.position.x;
    odom.y   = ros_odom.pose.pose.position.y;
    odom.yaw = tf2::getYaw(ros_odom.pose.pose.orientation);

    odom.vx = ros_odom.twist.twist.linear.x;
    odom.vy = ros_odom.twist.twist.linear.y;
    odom.wz = ros_odom.twist.twist.angular.z;

    odom.stamp = static_cast<float>(ros_odom.header.stamp.toSec());

    return odom;
}

} // namespace ros_utils