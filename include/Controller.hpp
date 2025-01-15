#ifndef __NANO_MPPIC_ROS_HPP__
#define __NANO_MPPIC_ROS_HPP__

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class NanoMPPIcROS : public nav_core::BaseLocalPlanner {

     // VARIABLES

    private:

        tf2_ros::Buffer* tf_;

        nano_mppic::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;

        nano_mppic::MPPIc nano_mppic_;

        nano_mppic::objects::Path global_plan_;
        nano_mppic::objects::Odometry2d current_odom_;

        bool initialized_;

        ros::Subscriber odom_sub_;

    // FUNCTIONS

    public:

        NanoMPPIcROS();
        NanoMPPIcROS(std::string name, 
                        tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);
        
        ~NanoMPPIcROS();

        void initialize(std::string name, 
                        tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros) override;

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) override;

        bool isGoalReached() override;

        bool is_initialized();

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

};

#endif