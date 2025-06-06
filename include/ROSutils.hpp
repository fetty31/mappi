/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Useful auxiliar functions to transform ROS types to MaPPI's and viceversa. 
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include "mppic.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/ColorRGBA.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mappi::ros_utils {

/**
 * @brief Change type from ROS path to mappi path
 * 
 * @param in_path 
 * @param out_path 
 */
void ros2mppic(const std::vector<geometry_msgs::PoseStamped>& in_path, 
                mappi::objects::Path& out_path)
{
    out_path.reset(in_path.size());

    for (size_t i=0; i < in_path.size(); ++i) {
        out_path.x(i) = in_path[i].pose.position.x;
        out_path.y(i) = in_path[i].pose.position.y;
        out_path.yaw(i) = tf2::getYaw(in_path[i].pose.orientation);
    }
}

/**
 * @brief Change type from ROS path to mappi path performing some transformation (change of frame)
 * 
 * @param in_path 
 * @param out_path 
 * @param transform 
 */
void ros2mppic(const std::vector<geometry_msgs::PoseStamped>& in_path, 
                mappi::objects::Path& out_path,
                geometry_msgs::TransformStamped& transform)
{
    out_path.reset(in_path.size());

    for (size_t i=0; i < in_path.size(); ++i) {
        geometry_msgs::PoseStamped pose_transformed;
        tf2::doTransform(in_path[i], pose_transformed, transform);
        out_path.x(i) = pose_transformed.pose.position.x;
        out_path.y(i) = pose_transformed.pose.position.y;
        out_path.yaw(i) = tf2::getYaw(pose_transformed.pose.orientation);
    }
}

/**
 * @brief Change type from ROS pose to mappi odom2D
 * 
 * @param in_pose 
 * @param out_pose 
 */
void ros2mppic(const geometry_msgs::PoseStamped& in_pose, 
                mappi::objects::Odometry2d& out_pose)
{
    out_pose.x   = in_pose.pose.position.x;
    out_pose.y   = in_pose.pose.position.y;
    out_pose.yaw = tf2::getYaw(in_pose.pose.orientation);
    out_pose.stamp = static_cast<float>(in_pose.header.stamp.toSec());
}

/**
 * @brief Change type from ROS odometry to mappi odom2D
 * 
 * @param in_odom 
 * @param out_odom 
 */
void ros2mppic(const nav_msgs::Odometry& in_odom,
                mappi::objects::Odometry2d& out_odom)
{
    out_odom.x   = in_odom.pose.pose.position.x;
    out_odom.y   = in_odom.pose.pose.position.y;
    out_odom.yaw = tf2::getYaw(in_odom.pose.pose.orientation);

    out_odom.vx = in_odom.twist.twist.linear.x;
    out_odom.vy = in_odom.twist.twist.linear.y;
    out_odom.wz = in_odom.twist.twist.angular.z;

    out_odom.stamp = static_cast<float>(in_odom.header.stamp.toSec());
}

/**
 * @brief Change type from mappi path to vector of ROS poses
 * 
 * @param in_path 
 * @param out_path 
 */
void mppic2ros(const mappi::objects::Path& in_path, 
                std::vector<geometry_msgs::PoseStamped>& out_path)
{
    out_path.reserve(in_path.x.size());

    for (size_t i=0; i < in_path.x.size(); ++i) {
        static geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = in_path.x(i);
        pose_msg.pose.position.y = in_path.y(i);

        tf2::Quaternion q;
        q.setRPY( 0, 0, in_path.yaw(i) ); 
        pose_msg.pose.orientation = tf2::toMsg(q);

        out_path.push_back(pose_msg);
    }
}

/**
 * @brief Change type from mappi path to ROS nav path
 * 
 * @param in_path 
 * @param out_path 
 */
void mppic2ros(const mappi::objects::Path& in_path, 
                nav_msgs::Path& out_path)
{
    out_path.poses.clear();
    out_path.poses.reserve(in_path.x.size());

    for (size_t i=0; i < in_path.x.size(); ++i) {
        static geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = in_path.x(i);
        pose_msg.pose.position.y = in_path.y(i);

        tf2::Quaternion q;
        q.setRPY( 0, 0, in_path.yaw(i) ); 
        pose_msg.pose.orientation = tf2::toMsg(q);

        out_path.poses.push_back(pose_msg);
    }
}

/**
 * @brief Change type from mappi path to ROS nav path including frame_id arg
 * 
 * @param in_path 
 * @param out_path 
 * @param frame_id 
 */
void mppic2ros(const mappi::objects::Path& in_path, 
                nav_msgs::Path& out_path,
                std::string frame_id)
{
    out_path.poses.clear();
    out_path.poses.reserve(in_path.x.size());

    for (size_t i=0; i < in_path.x.size(); ++i) {
        static geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = in_path.x(i);
        pose_msg.pose.position.y = in_path.y(i);

        tf2::Quaternion q;
        q.setRPY( 0, 0, in_path.yaw(i) ); 
        pose_msg.pose.orientation = tf2::toMsg(q);

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = frame_id;

        out_path.poses.push_back(pose_msg);
    }

    out_path.header.frame_id = frame_id;
    out_path.header.stamp = ros::Time::now();
}

/**
 * @brief Create a Pose object from 3D position
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return geometry_msgs::Pose 
 */
inline geometry_msgs::Pose createPose(double x, double y, double z)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    return pose;
}

/**
 * @brief Create a Pose object from 3D position and 2D orientation
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param yaw 
 * @return geometry_msgs::Pose 
 */
inline geometry_msgs::Pose createPose(double x, double y, double z, double yaw)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY( 0, 0, yaw );
    q.normalize();

    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
}

/**
 * @brief Create a Scale object
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return geometry_msgs::Vector3 
 */
inline geometry_msgs::Vector3 createScale(double x, double y, double z)
{
    geometry_msgs::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
}

/**
 * @brief Create a Color object
 * 
 * @param r 
 * @param g 
 * @param b 
 * @param a 
 * @return std_msgs::ColorRGBA 
 */
inline std_msgs::ColorRGBA createColor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

/**
 * @brief Create a Marker object
 * 
 * @param id 
 * @param pose 
 * @param scale 
 * @param color 
 * @param frame_id 
 * @param ns 
 * @return visualization_msgs::Marker 
 */
inline visualization_msgs::Marker createMarker( 
    int id, const geometry_msgs::Pose & pose, const geometry_msgs::Vector3 & scale,
    const std_msgs::ColorRGBA & color, const std::string & frame_id, const std::string ns="mappi_traj")
{
    using visualization_msgs::Marker;
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;

    marker.pose = pose;
    marker.scale = scale;
    marker.color = color;
    return marker;
}

/**
 * @brief Create a Arrow Marker object
 * 
 * @param id 
 * @param pose 
 * @param scale 
 * @param color 
 * @param frame_id 
 * @param ns 
 * @return visualization_msgs::Marker 
 */
inline visualization_msgs::Marker createArrowMarker( 
    int id, const geometry_msgs::Pose & pose, const geometry_msgs::Vector3 & scale,
    const std_msgs::ColorRGBA & color, const std::string & frame_id, const std::string ns="mappi_traj")
{
    using visualization_msgs::Marker;
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;

    marker.pose = pose;
    marker.scale = scale;
    marker.color = color;
    return marker;
}

} // namespace mappi::ros_utils