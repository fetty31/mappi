#ifndef __MAPPI_ODOM_HELPER_HPP__
#define __MAPPI_ODOM_HELPER_HPP__

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <nav_msgs/Odometry.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/bind.hpp>

namespace mappi {

class OdomHelper {
    public:
        OdomHelper(const std::string& name){
            ros::NodeHandle nh("~"+name);

            std::string steer_topic, odom_topic;
            nh.param<std::string>("Odometry/steering_topic", steer_topic, "");
            nh.param<std::string>("Odometry/odom_topic", odom_topic, "");
            
            if(steer_topic != "")
                steering_sub = nh.subscribe<ackermann_msgs::AckermannDriveStamped>(steer_topic, 1, 
                                    boost::bind( &mappi::OdomHelper::steering_callback, this, _1 ));
            if(odom_topic != "")
                odom_sub     = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1,  
                                    boost::bind( &mappi::OdomHelper::odom_callback, this, _1 ));
        }

        void steering_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
        {
            const std::lock_guard<std::mutex> lock(odom_mtx);

            odometry_.steering = msg->drive.steering_angle;
            odometry_.wz = msg->drive.steering_angle_velocity;
            /*NOTE:
                when using BicycleKin model we abuse notation so that wz is in fact the steering rate
                instead of the angular velocity of the robot
            */
        }

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
        }

        mappi::objects::Odometry2d getOdometry(){
            return odometry_;
        }

        void fillTwist(mappi::objects::Odometry2d& odom){
            odom.vx = odometry_.vx;
            odom.vy = odometry_.vy;
            odom.wz = odometry_.wz;
        }

        void fillPose(mappi::objects::Odometry2d& odom){
            odom.x = odometry_.x;
            odom.y = odometry_.y;
            odom.yaw = odometry_.yaw;
        }

        void fillTwistAndSteering(mappi::objects::Odometry2d& odom){
            fillTwist(odom);
            odom.steering = odometry_.steering;
        }

        void fillPoseAndSteering(mappi::objects::Odometry2d& odom){
            fillPose(odom);
            odom.steering = odometry_.steering;
        }

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