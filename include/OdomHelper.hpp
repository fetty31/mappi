/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Auxiliar class to receive odom data.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_NAV2_CONTROLLER_ODOM_HELPER_HPP__
#define __MAPPI_NAV2_CONTROLLER_ODOM_HELPER_HPP__

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/bind.hpp>

namespace mappi {

class OdomHelper {
    public:
        /**
         * @brief Construct a new Odom Helper object
         * 
         * @param name 
         */
        OdomHelper(const std::string& name){
            ros::NodeHandle nh_upper;

            steering_sub = nh_upper.subscribe<ackermann_msgs::AckermannDriveStamped>("steering_feedback", 1, 
                                        boost::bind( &mappi::OdomHelper::steering_callback, this, _1 ));
            ROS_INFO("mappi::ODOM_HELPER: subscribed to steering topic: %s", steering_sub.getTopic().c_str());

            odom_sub     = nh_upper.subscribe<nav_msgs::Odometry>("odom", 1,  
                                        boost::bind( &mappi::OdomHelper::odom_callback, this, _1 ));

            ROS_INFO("mappi::ODOM_HELPER: subscribed to odom topic: %s", odom_sub.getTopic().c_str());
        }

        /**
         * @brief Process received steering message
         * 
         * @param msg 
         */
        void steering_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
        {
            const std::lock_guard<std::mutex> lock(odom_mtx);

            odometry_.steering = msg->drive.steering_angle;
            odometry_.wz = msg->drive.steering_angle_velocity;
            /*NOTE:
                when using BicycleKin model we abuse notation so that wz is in fact the steering rate
                instead of the angular velocity of the robot
            */

            ROS_INFO_ONCE("mappi::ODOM_HELPER: steering callback");
        }

        /**
         * @brief Process received odometry message
         * 
         * @param msg 
         */
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            const std::lock_guard<std::mutex> lock(odom_mtx);

            odometry_.x = msg->pose.pose.position.x;
            odometry_.y = msg->pose.pose.position.y;
            odometry_.yaw = tf2::getYaw(msg->pose.pose.orientation);

            odometry_.vx = msg->twist.twist.linear.x;
            odometry_.vy = msg->twist.twist.linear.y;
            // odometry_.wz = msg->twist.twist.angular.z;
            /*NOTE:
                if using Ackermann/Holonomic motion models, comment out the line above
            */

            ROS_INFO_ONCE("mappi::ODOM_HELPER: odometry callback");
        }

        /**
         * @brief Get the Odometry object
         * 
         * @return mappi::objects::Odometry2d 
         */
        mappi::objects::Odometry2d getOdometry(){
            return odometry_;
        }

        /**
         * @brief Fill mappi odometry velocity
         * 
         * @param odom 
         */
        void fillTwist(mappi::objects::Odometry2d& odom){
            odom.vx = odometry_.vx;
            odom.vy = odometry_.vy;
            odom.wz = odometry_.wz;
        }

        /**
         * @brief Fill mappi odometry pose
         * 
         * @param odom 
         */
        void fillPose(mappi::objects::Odometry2d& odom){
            odom.x = odometry_.x;
            odom.y = odometry_.y;
            odom.yaw = odometry_.yaw;
        }

        /**
         * @brief Fill mappi odometry steering angle
         * 
         * @param odom 
         */
        void fillSteering(mappi::objects::Odometry2d& odom){
            odom.steering = odometry_.steering;
            odom.wz = odometry_.wz; // only in case BycicleKin model is active
        }

        /**
         * @brief Fill mappi odometry with velocity and steering
         * 
         * @param odom 
         */
        void fillTwistAndSteering(mappi::objects::Odometry2d& odom){
            fillTwist(odom);
            fillSteering(odom);
        }

        /**
         * @brief Fill mappi odometry with pose and steering
         * 
         * @param odom 
         */
        void fillPoseAndSteering(mappi::objects::Odometry2d& odom){
            fillPose(odom);
            fillSteering(odom);
        }

        /**
         * @brief Fill mappi odometry completely
         * 
         * @param odom 
         */
        void fillOdometry(mappi::objects::Odometry2d& odom){
            fillPose(odom);
            fillTwistAndSteering(odom);
        }

    private:
        ros::Subscriber steering_sub;
        ros::Subscriber odom_sub;

        std::mutex odom_mtx;

        mappi::objects::Odometry2d odometry_;
};

} // namespace mappi

#endif