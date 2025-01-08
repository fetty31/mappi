#pragma once

#include "nano_mppic/include/Predictor.hpp"

#include <nav_core/base_local_planner.h>

/* To-Do:
    - define ROS wrapper of nano_mppic
    - plugin for ros_nav (BaseLocalPlanner in Noetic)
*/

class NanoMPPIcROS : public nav_core::BaseLocalPlanner {

     // VARIABLES

    private:

        tf2_ros::Buffer* tf_;

        costmap_2d::Costmap2DROS* costmap_ros_;

        nano_mppic::Predictor nano_mppic_;

        bool initialized_;

    // FUNCTIONS

    public:

        NanoMPPIcROS();
        ~NanoMPPIcROS();

        void initialize(std::string name, 
                        tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);

        void computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

        bool isGoalReached();

        bool is_initialized();

};