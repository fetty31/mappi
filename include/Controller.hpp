#ifndef __MAPPI_ROS_HPP__
#define __MAPPI_ROS_HPP__

#include <chrono>
#include <cmath>

#include "mppic.hpp"

#include "ROSutils.hpp"
#include "Visualizer.hpp"
#include "OdomHelper.hpp"

#include <dynamic_reconfigure/server.h>
#include <mappi/MPPIPlannerROSConfig.h>

#include <nav_core/base_local_planner.h>

#ifdef HAS_NAVFN
    #include "NavFnWrapper.hpp"
#endif

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

namespace mappi {

class MPPIcROS : public nav_core::BaseLocalPlanner {

     // VARIABLES

    private:

        tf2_ros::Buffer* tf_;
        std::string local_frame_, global_frame_;

        mappi::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        ros::ServiceClient costmap_client_;

        MPPIc mappi_;
        std::unique_ptr<Visualizer> visualizer_ptr_;
        std::unique_ptr<OdomHelper> odom_helper_ptr_;

        #ifdef HAS_NAVFN
            NavFnWrapper navfn_wrapper_;
        #endif

        objects::Path global_plan_;
        objects::Path local_plan_;

        objects::Odometry2d current_odom_;

        bool initialized_;
        bool use_local_planner_;

        float goal_tolerance_;
        float dist_shift_;

        ros::Publisher global_pub_;
        ros::Publisher local_pub_;


        dynamic_reconfigure::Server<mappi::MPPIPlannerROSConfig> *dyn_srv_;

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

        void reconfigure_callback(mappi::MPPIPlannerROSConfig &dyn_cfg, uint32_t level);

};

} // namespace mappi

#endif