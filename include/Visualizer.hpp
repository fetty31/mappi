/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   The Visualizer class is a visualization tool for publishing useful insights about MaPPI's outputs.
 *   It owns a pointer to the MPPIc object, which uses to retrieve all data to visualize.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_NAV2_CONTROLLER_VISUALIZER_HPP__
#define __MAPPI_NAV2_CONTROLLER_VISUALIZER_HPP__

#include <memory>
#include <string>
#include <thread>

#include "mppic.hpp"
#include "ROSutils.hpp"
#include "ParametersHandler.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mappi {

class Visualizer {

    // VARIABLES

    private:
        MPPIc* mppic_;
        ParametersHandler* parameters_handler_;

        rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
        std::string name_;

        rclcpp::Clock::SharedPtr clock_;

        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_, marker_opt_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pcl_pub_, pcl_opt_pub_;

        config::Visualization cfg_;

        int marker_id_;
        std::string frame_id_;

    // FUNCTIONS

    public:

        /**
         * @brief Construct a new Visualizer object
         * 
         */
        Visualizer() = default;

        /**
         * @brief Configure a new Visualizer object
         * 
         * @param mppic Pointer to MPPI controller
         * @param config Configuration 
         */
        void on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, 
                    const std::string &name,
                    MPPIc* mppic,
                    config::Visualization& config,
                    ParametersHandler* parameters_handler);

        /**
         * @brief Clean up the Visualizer object
         * 
         */
        void on_cleanup();

        /**
         * @brief Activate the Visualizer object
         * 
         */
        void on_activate();

        /**
         * @brief Deactivate the Visualizer object
         * 
         */
        void on_deactivate();
        
        /**
         * @brief Publish visualization topics
         * 
         */
        void publish();

    private:

        /**
         * @brief Fill ROS marker array msg from sampled trajectories
         * 
         * @param trajectories Sampled trajectories
         * @param batch_stride Batch size stride for avoiding too much overhead
         * @param time_stride Horizon length stride for avoiding too much overhead
         * @param msg Output ROS message
         */
        void fillMarkermsg(const objects::Trajectory& trajectories,
                            const int &batch_stride,
                            const int &time_stride,
                            visualization_msgs::msg::MarkerArray* msg);

        /**
        * @brief Fill ROS point cloud msg from sampled trajectories
        * 
        * @param trajectories Sampled trajectories
        * @param batch_stride Batch size stride for avoiding too much overhead
        * @param time_stride Horizon length stride for avoiding too much overhead
        * @param msg Output ROS message
        */
        void fillPointCloudmsg(const objects::Trajectory& trajectories,
                                const int &batch_stride,
                                const int &time_stride,
                                sensor_msgs::msg::PointCloud2* msg);
        
        /**
         * @brief Get the marker array msg
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getMarkerTrajectories(visualization_msgs::msg::MarkerArray* msg);

        /**
         * @brief Get the marker array msg filled only with the optimal trajectory
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getMarkerOptimalTrajectory(visualization_msgs::msg::MarkerArray* msg);

        /**
         * @brief Get the point cloud msg
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getPointCloudTrajectories(sensor_msgs::msg::PointCloud2* msg);

        /**
         * @brief Get the point cloud msg filled only with the optimal trajectory
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getPointCloudOptimalTrajectory(sensor_msgs::msg::PointCloud2* msg);
        
        /**
         * @brief Reset visualizer object
         * 
         */
        void reset();

};

void Visualizer::on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, 
                                const std::string &name,
                                MPPIc* mppic,
                                config::Visualization& config,
                                ParametersHandler* parameters_handler)
{
    mppic_ = mppic;
    parameters_handler_ = parameters_handler;
    parent_ = parent;
    name_ = name;
    cfg_ = config;

    auto node = parent_.lock();
    if (!node) {
        RCLCPP_ERROR(rclcpp::get_logger("Visualizer"), "Node not available during on_configure!");
        return;
    }

    clock_ = node->get_clock();

    auto getParam = parameters_handler_->getParamGetter(name_ + ".GeneralSettings"); 
    getParam(frame_id_, "global_frame", std::string(""), ParameterType::Static);

    reset();

    marker_pub_     = node->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 1);
    marker_opt_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("optimal_trajectory", 1);

    pcl_pub_       = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_trajectories", 1);
    pcl_opt_pub_   = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_optimal_trajectory", 1);
}

void Visualizer::on_cleanup()
{
    marker_pub_.reset();   
    marker_opt_pub_.reset();   
    pcl_pub_.reset();   
    pcl_opt_pub_.reset();
}

void Visualizer::on_activate()
{
    marker_pub_->on_activate();   
    marker_opt_pub_->on_activate();   
    pcl_pub_->on_activate();   
    pcl_opt_pub_->on_activate();   
}

void Visualizer::on_deactivate()
{
    marker_pub_->on_deactivate();   
    marker_opt_pub_->on_deactivate();   
    pcl_pub_->on_deactivate();   
    pcl_opt_pub_->on_deactivate();   
}

void Visualizer::publish()
{
    if(!cfg_.active)
        return;

    sensor_msgs::msg::PointCloud2 pcl_msg; 
    visualization_msgs::msg::MarkerArray m_msg;

    // if( (marker_pub_->getNumSubscribers() > 0) && getMarkerTrajectories(&m_msg) ) {
    if( getMarkerTrajectories(&m_msg) ) {
        marker_pub_->publish(m_msg);
    }

    // if( (pcl_pub_->getNumSubscribers() > 0) && getPointCloudTrajectories(&pcl_msg) ) {
    if( getPointCloudTrajectories(&pcl_msg) ) {
        pcl_pub_->publish(pcl_msg);
    }

    // if( (marker_opt_pub_->getNumSubscribers() > 0) && getMarkerOptimalTrajectory(&m_msg) ) {
    if( getMarkerOptimalTrajectory(&m_msg) ) {
        marker_opt_pub_->publish(m_msg);
    }

    // if( (pcl_opt_pub_->getNumSubscribers() > 0) && getPointCloudOptimalTrajectory(&pcl_msg) ) {
    if( getPointCloudOptimalTrajectory(&pcl_msg) ) {
        pcl_opt_pub_->publish(pcl_msg);
    }

    reset();
}

bool Visualizer::getMarkerTrajectories(visualization_msgs::msg::MarkerArray* msg)
{
    const objects::Trajectory trajectories = mppic_->getCandidateTrajectories();

    auto & shape = trajectories.x.shape();
    if(shape[0] < 1)
        return false;
    
    fillMarkermsg(trajectories, cfg_.batch_stride, cfg_.time_stride, msg);

    return true;
}

bool Visualizer::getMarkerOptimalTrajectory(visualization_msgs::msg::MarkerArray* msg)
{
    const objects::Trajectory trajectory = mppic_->getOptimalTrajectory();

    auto & shape = trajectory.x.shape();
    if(shape[1] < 1)
        return false;
    
    fillMarkermsg(trajectory, cfg_.batch_stride, 1 /*time_stride*/, msg);

    return true;
}

bool Visualizer::getPointCloudTrajectories(sensor_msgs::msg::PointCloud2* msg)
{
    const objects::Trajectory trajectories = mppic_->getCandidateTrajectories();

    auto & shape = trajectories.x.shape();
    if(shape[0] < 1)
        return false;
    
    fillPointCloudmsg(trajectories, cfg_.batch_stride, cfg_.time_stride, msg);

    return true;
}

bool Visualizer::getPointCloudOptimalTrajectory(sensor_msgs::msg::PointCloud2* msg)
{
    const objects::Trajectory trajectory = mppic_->getOptimalTrajectory();

    auto & shape = trajectory.x.shape();
    if(shape[1] < 1)
        return false;
    
    fillPointCloudmsg(trajectory, cfg_.batch_stride, 1 /*cfg_.time_stride*/, msg);

    return true;
}


void Visualizer::reset()
{
    marker_id_ = 0;
}

void Visualizer::fillMarkermsg(const objects::Trajectory& trajectories,
                                const int &batch_stride,
                                const int &time_stride,
                                visualization_msgs::msg::MarkerArray* msg)
{
    msg->markers.clear();

    auto & shape = trajectories.x.shape();
    const float shape_1 = static_cast<float>(shape[1]);
    if(shape[0] > 1)
        msg->markers.reserve(floor(shape[0] / batch_stride) * floor(shape[1] / time_stride));
    else
        msg->markers.reserve(floor(shape[1] / time_stride));

    for (size_t i = 0; i < shape[0]; i += batch_stride) {
        for (size_t j = 0; j < shape[1]; j += time_stride) {
            const float j_flt = static_cast<float>(j);
            float blue_component = 1.0f - j_flt / shape_1;
            float green_component = j_flt / shape_1;

            auto color = ros_utils::createColor(0, green_component, blue_component, 1);

            auto pose = (shape[0] > 1) ? ros_utils::createPose(trajectories.x(i, j), trajectories.y(i, j), cfg_.default_z) :
                                         ros_utils::createPose(trajectories.x(i, j), trajectories.y(i, j), cfg_.default_z, trajectories.yaw(i, j));

            auto scale  = (shape[0] > 1) ? ros_utils::createScale(cfg_.scale, cfg_.scale, cfg_.scale) :
                                           ros_utils::createScale(cfg_.scale, cfg_.scale/3.0f, cfg_.scale/3.0f);

            auto marker = (shape[0] > 1) ? ros_utils::createMarker(marker_id_++, pose, scale, color, clock_, frame_id_) :
                                           ros_utils::createArrowMarker(marker_id_++, pose, scale, color, clock_, frame_id_);

            msg->markers.push_back(marker);
        }
    }
}

void Visualizer::fillPointCloudmsg(const objects::Trajectory& trajectories, 
                                    const int &batch_stride,
                                    const int &time_stride,
                                    sensor_msgs::msg::PointCloud2* msg)
{
    auto & shape = trajectories.x.shape();
    const float shape_0 = static_cast<float>(shape[0]);
    const float shape_1 = static_cast<float>(shape[1]);

    //Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(*msg);

    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

    //Msg header
    msg->header = std_msgs::msg::Header();
    msg->header.stamp = clock_->now();
    msg->header.frame_id = frame_id_;

    msg->height = 1;
    msg->width = (shape_0 > 1) ? ceil(shape_0 / batch_stride) * ceil(shape_1 / time_stride) : ceil(shape_1 / time_stride);
    msg->is_dense = true;

    //Total number of bytes per point
    msg->point_step = 16;
    msg->row_step = msg->point_step * msg->width;

    // Set up data
    msg->data.clear();
    msg->data.resize(msg->row_step);

    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(*msg, "intensity");

    //iterate over the message and populate the fields.  
    for (size_t i = 0; i < shape[0]; i += batch_stride) {
        for (size_t j = 0; j < shape[1]; j += time_stride) {
            *iter_x = trajectories.x(i, j);
            *iter_y = trajectories.y(i, j);
            *iter_z = cfg_.default_z;

            const float j_flt = static_cast<float>(j);
            float intensity = 1.0f - j_flt / shape_1;

            *iter_i = intensity;

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_i;
        }
    }
}

} // namespace mappi


#endif