#ifndef __MAPPI_VISUALIZER_HPP__
#define __MAPPI_VISUALIZER_HPP__

#include <memory>
#include <string>
#include <thread>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace mappi {

class Visualizer {

    // VARIABLES

    private:
        MPPIc* mppic_;

    	ros::NodeHandle* nh_ptr_;
        ros::Publisher marker_pub_, marker_opt_pub_;
        ros::Publisher pcl_pub_, pcl_opt_pub_;

        std::thread pub_thread_;

        int batch_stride_;
        int time_stride_;

        float default_z_;
        float scale_;

        int marker_id_;
        std::string frame_id_;


    // FUNCTIONS

    public:

        /**
         * @brief Construct a new Visualizer object
         * 
         * @param mppic Pointer to MPPI controller
         * @param nh_ptr Pointer to ROS node handle
         */
        Visualizer(MPPIc* mppic, ros::NodeHandle* nh_ptr);

        /**
         * @brief Destroy the Visualizer object
         * 
         */
        ~Visualizer();

        /**
         * @brief Start visualizing thread
         * 
         */
        void startThread();

        /**
         * @brief Stop visualizing thread
         * 
         */
        void stopThread();

        /**
         * @brief Publish visualization topics
         * 
         */
        void publish();

    private:

        /**
         * @brief Publish thread
         * 
         */
        void publishThread();

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
                            visualization_msgs::MarkerArray* msg);

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
                                sensor_msgs::PointCloud2* msg);
        
        /**
         * @brief Get the marker array msg
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getMarkerTrajectories(visualization_msgs::MarkerArray* msg);

        /**
         * @brief Get the marker array msg filled only with the optimal trajectory
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getMarkerOptimalTrajectory(visualization_msgs::MarkerArray* msg);

        /**
         * @brief Get the point cloud msg
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getPointCloudTrajectories(sensor_msgs::PointCloud2* msg);

        /**
         * @brief Get the point cloud msg filled only with the optimal trajectory
         * 
         * @param msg Output ROS message
         * @return true 
         * @return false 
         */
        bool getPointCloudOptimalTrajectory(sensor_msgs::PointCloud2* msg);
        
        /**
         * @brief Reset visualizer object
         * 
         */
        void reset();

};

Visualizer::Visualizer(MPPIc* mppic, 
                        ros::NodeHandle* nh_ptr) : mppic_(mppic),
                                                    nh_ptr_(nh_ptr)
{
    nh_ptr->param<int>("Visualization/batch_stride", batch_stride_, 50); 
    nh_ptr->param<int>("Visualization/time_stride",  time_stride_,  3); 
    nh_ptr->param<float>("Visualization/default_z",  default_z_,    0.03f); 
    nh_ptr->param<float>("Visualization/scale",      scale_,        0.03f); 

    nh_ptr->param<std::string>("GeneralSettings/global_frame", frame_id_, ""); 
    if(frame_id_ == ""){
        ros::NodeHandle nh_upper("~");
        nh_upper.param<std::string>("local_costmap/global_frame", frame_id_, "odom");
    }

    reset();

    marker_pub_      = nh_ptr_->advertise<visualization_msgs::MarkerArray>("trajectories", 1);
    marker_opt_pub_  = nh_ptr_->advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1);

    pcl_pub_     = nh_ptr_->advertise<sensor_msgs::PointCloud2>("pcl_trajectories", 1);
    pcl_opt_pub_ = nh_ptr_->advertise<sensor_msgs::PointCloud2>("pcl_optimal_trajectory", 1);
}

Visualizer::~Visualizer()
{
    delete mppic_;
    delete nh_ptr_;
}

void Visualizer::startThread()
{
    pub_thread_ = std::thread(std::bind(&Visualizer::publishThread, this));
}

void Visualizer::stopThread()
{
    if (pub_thread_.joinable())
        pub_thread_.join();
}

void Visualizer::publish()
{
    sensor_msgs::PointCloud2 pcl_msg; 
    visualization_msgs::MarkerArray m_msg;

    if( (marker_pub_.getNumSubscribers() > 0) && getMarkerTrajectories(&m_msg) ) {
        marker_pub_.publish(m_msg);
    }

    if( (pcl_pub_.getNumSubscribers() > 0) && getPointCloudTrajectories(&pcl_msg) ) {
        pcl_pub_.publish(pcl_msg);
    }

    if( (marker_opt_pub_.getNumSubscribers() > 0) && getMarkerOptimalTrajectory(&m_msg) ) {
        marker_opt_pub_.publish(m_msg);
    }

    if( (pcl_opt_pub_.getNumSubscribers() > 0) && getPointCloudOptimalTrajectory(&pcl_msg) ) {
        pcl_opt_pub_.publish(pcl_msg);
    }

    reset();
}

void Visualizer::publishThread()
{
    ROS_INFO_ONCE("mappi::Visualizer publish thread started!");

    while(ros::ok()){

        publish();

        ros::Duration(0.05).sleep();
    }
}

bool Visualizer::getMarkerTrajectories(visualization_msgs::MarkerArray* msg)
{
    const objects::Trajectory trajectories = mppic_->getCandidateTrajectories();

    auto & shape = trajectories.x.shape();
    if(shape[0] < 1)
        return false;
    
    fillMarkermsg(trajectories, batch_stride_, time_stride_, msg);

    return true;
}

bool Visualizer::getMarkerOptimalTrajectory(visualization_msgs::MarkerArray* msg)
{
    const objects::Trajectory trajectory = mppic_->getOptimalTrajectory();

    auto & shape = trajectory.x.shape();
    if(shape[1] < 1)
        return false;
    
    fillMarkermsg(trajectory, batch_stride_, 1 /*time_stride_*/, msg);

    return true;
}

bool Visualizer::getPointCloudTrajectories(sensor_msgs::PointCloud2* msg)
{
    const objects::Trajectory trajectories = mppic_->getCandidateTrajectories();

    auto & shape = trajectories.x.shape();
    if(shape[0] < 1)
        return false;
    
    fillPointCloudmsg(trajectories, batch_stride_, time_stride_, msg);

    return true;
}

bool Visualizer::getPointCloudOptimalTrajectory(sensor_msgs::PointCloud2* msg)
{
    const objects::Trajectory trajectory = mppic_->getOptimalTrajectory();

    auto & shape = trajectory.x.shape();
    if(shape[1] < 1)
        return false;
    
    fillPointCloudmsg(trajectory, batch_stride_, 1 /*time_stride_*/, msg);

    return true;
}


void Visualizer::reset()
{
    marker_id_ = 0;
}

void Visualizer::fillMarkermsg(const objects::Trajectory& trajectories,
                                const int &batch_stride,
                                const int &time_stride,
                                visualization_msgs::MarkerArray* msg)
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

            auto pose = (shape[0] > 1) ? ros_utils::createPose(trajectories.x(i, j), trajectories.y(i, j), default_z_) :
                                         ros_utils::createPose(trajectories.x(i, j), trajectories.y(i, j), default_z_, trajectories.yaw(i, j));

            auto scale  = (shape[0] > 1) ? ros_utils::createScale(scale_, scale_, scale_) :
                                           ros_utils::createScale(scale_, scale_/3.0f, scale_/3.0f);

            auto marker = (shape[0] > 1) ? ros_utils::createMarker(marker_id_++, pose, scale, color, frame_id_) :
                                           ros_utils::createArrowMarker(marker_id_++, pose, scale, color, frame_id_);

            msg->markers.push_back(marker);
        }
    }
}

void Visualizer::fillPointCloudmsg(const objects::Trajectory& trajectories, 
                                    const int &batch_stride,
                                    const int &time_stride,
                                    sensor_msgs::PointCloud2* msg)
{
    auto & shape = trajectories.x.shape();
    const float shape_0 = static_cast<float>(shape[0]);
    const float shape_1 = static_cast<float>(shape[1]);

    //Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(*msg);

    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);

    //Msg header
    msg->header = std_msgs::Header();
    msg->header.stamp = ros::Time::now();
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
    int c=0;
    for (size_t i = 0; i < shape[0]; i += batch_stride) {
        for (size_t j = 0; j < shape[1]; j += time_stride) {
            *iter_x = trajectories.x(i, j);
            *iter_y = trajectories.y(i, j);
            *iter_z = default_z_;

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