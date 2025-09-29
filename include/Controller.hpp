/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   MPPIcROS is the default class adhering to the pluginlib C++ standard.
 *   It handles all the interaction between MaPPI controller and ROS move_base pipeline.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_NAV2_CONTROLLER_ROS_HPP__
#define __MAPPI_NAV2_CONTROLLER_ROS_HPP__

#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <memory>

#include "mppic.hpp"

#include "ROSutils.hpp"
#include "Visualizer.hpp"
#include "OdomHelper.hpp"
#include "ParametersHandler.hpp"

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"

// #include <std_srvs/Empty.h>

namespace mappi {

class MPPIcROS : public nav2_core::Controller {

     // VARIABLES

    protected:

        nav2::LifecycleNode::WeakPtr node_;

        rclcpp::Clock::SharedPtr clock_;

        std::shared_ptr<tf2_ros::Buffer> tf_;

        std::string local_frame_, global_frame_;
        std::string plugin_name_;

        mappi::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        mappi::shared_ptr<mappi::utils::CostmapInterface> costmap_mappi_;

        rclcpp::Logger logger_ {rclcpp::get_logger("MaPPIController")};

        MPPIc mappi_;
        config::MPPIc config_;
        std::unique_ptr<Visualizer> visualizer_ptr_;
        std::unique_ptr<ParametersHandler> parameters_handler_;
        // std::unique_ptr<OdomHelper> odom_helper_ptr_;

        nav_msgs::msg::Path global_plan_;

        objects::Odometry2d current_odom_;

        bool initialized_{false};

        rclcpp::Duration transform_tolerance_ {0, 0};

        std::shared_ptr<nav2::Publisher<nav_msgs::msg::Path>> global_pub_;
        std::shared_ptr<nav2::Publisher<nav_msgs::msg::Path>> local_pub_;

        // ros::ServiceClient costmap_client_;

    // FUNCTIONS

    public:

        /**
         * @brief Construct a new MPPIcROS object
         * 
         */
        MPPIcROS() = default;

        /**
         * @brief Destroy the MPPIcROS object
         * 
         */
        ~MPPIcROS() override;
        
        /**
         * @brief Configure MPPIcROS object
         * 
         * @param name 
         * @param tf 
         * @param costmap_ros 
         */
        void configure( const nav2::LifecycleNode::WeakPtr & parent,
                        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        
        /**
         * @brief Perform cleanup
         * 
         */
        void cleanup() override;

        /**
         * @brief Activate controller
         * 
         */
        void activate() override;

        /**
         * @brief Deactivate controller
         * 
         */
        void deactivate() override;

        /**
         * @brief Set speed limit
         * 
         * @param speed_limit 
         * @param percentage 
         */
        void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
        
        /**
         * @brief Compute velocity commands from MPPI controller
         * 
         * @param pose Robot pose
         * @param velocity Robot velocity
         * @param goal_checker Nav2 goal checker
         */
        geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                                                 const geometry_msgs::msg::Twist & velocity,
                                                                 nav2_core::GoalChecker* goal_checker) override;
        
        /**
         * @brief Set the global plan object
         * 
         * @param path 
         */
        void setPlan(const nav_msgs::msg::Path & path) override;
        
        /**
         * @brief Check if the goal is reached
         * 
         * @return true 
         * @return false 
         */
        bool isGoalReached() override;
        
        /**
         * @brief Check if MPPIcROS is initialized
         * 
         * @return true 
         * @return false 
         */
        bool is_initialized();

    protected:

        void setUpParameters(config::MPPIc& config);
            
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

        bool transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                           const std::string frame,
                           const geometry_msgs::msg::PoseStamped & in_pose,
                           geometry_msgs::msg::PoseStamped & out_pose,
                           const rclcpp::Duration & transform_tolerance) const;
};

} // namespace mappi

#endif
