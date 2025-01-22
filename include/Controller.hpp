#ifndef __NANO_MPPIC_ROS_HPP__
#define __NANO_MPPIC_ROS_HPP__

#include "mppic.hpp"
#include "ROSutils.hpp"
#include "Visualizer.hpp"

#include <nano_mppic/MPPIPlannerROSConfig.h>
#include <dynamic_reconfigure/server.h>

#include <chrono>

#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace nano_mppic {

class MPPIcROS : public nav_core::BaseLocalPlanner {

     // VARIABLES

    private:

        tf2_ros::Buffer* tf_;

        shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;

        MPPIc nano_mppic_;
        std::unique_ptr<Visualizer> vis_ptr_;

        objects::Path global_plan_;
        objects::Odometry2d current_odom_;

        bool initialized_;

        ros::Subscriber odom_sub_;

        dynamic_reconfigure::Server<nano_mppic::MPPIPlannerROSConfig> *dyn_srv_;

    // FUNCTIONS

    public:

        MPPIcROS();
        MPPIcROS(std::string name, 
                        tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);
        
        ~MPPIcROS();

        void initialize(std::string name, 
                        tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* costmap_ros) override;

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) override;

        bool isGoalReached() override;

        bool is_initialized();

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

        void reconfigure_callback(nano_mppic::MPPIPlannerROSConfig &dyn_cfg, uint32_t level);

};

} // namespace nano_mppic

#endif