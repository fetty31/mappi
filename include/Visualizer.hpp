#ifndef __NANO_MPPIC_VISUALIZER_HPP__
#define __NANO_MPPIC_VISUALIZER_HPP__

#include <memory>
#include <string>
#include <thread>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace nano_mppic {

class Visualizer {

    // VARIABLES

    private:
        MPPIc* mppic_;

    	ros::NodeHandle* nh_ptr_;
        ros::Publisher traj_pub_;

        std::unique_ptr<visualization_msgs::MarkerArray> points_;
        std::thread pub_thread_;

        int batch_stride_;
        int time_stride_;

        int marker_id_;
        std::string frame_id_;


    // FUNCTIONS

    public:

        Visualizer(MPPIc* mppic, ros::NodeHandle* nh_ptr);

        void publishThread();

    private:

        bool fillROSmsg();

        void reset();

};

Visualizer::Visualizer(MPPIc* mppic, 
                        ros::NodeHandle* nh_ptr) : mppic_(mppic),
                                                    nh_ptr_(nh_ptr),
                                                    batch_stride_(50),
                                                    time_stride_(3)
{
    /* To-Do:
        - get ROS params
    */
    frame_id_ = "ona2/odom";

    reset();

    traj_pub_ = nh_ptr_->advertise<visualization_msgs::MarkerArray>("/nano_mppic/trajectories", 1);

    pub_thread_ = std::thread(std::bind(&Visualizer::publishThread, this));
}

void Visualizer::publishThread()
{
    ROS_INFO_ONCE("NANO_MPPIC::Visualizer publish thread started!");

    while(ros::ok()){

        if(not fillROSmsg()) // no trajectory computed yet
            return;

        if (traj_pub_.getNumSubscribers() > 0) {
            traj_pub_.publish(*points_);
        }

        reset();
    }
}

void Visualizer::reset()
{
    marker_id_ = 0;
    points_ = std::make_unique<visualization_msgs::MarkerArray>();
}

bool Visualizer::fillROSmsg()
{

    const objects::Trajectory trajectories = mppic_->getCandidateTrajectories();

    auto & shape = trajectories.x.shape();
    if(shape[0] < 1)
        return false;

    const float shape_1 = static_cast<float>(shape[1]);
    points_->markers.reserve(floor(shape[0] / batch_stride_) * floor(shape[1] * time_stride_));

    for (size_t i = 0; i < shape[0]; i += batch_stride_) {
        for (size_t j = 0; j < shape[1]; j += time_stride_) {
            const float j_flt = static_cast<float>(j);
            float blue_component = 1.0f - j_flt / shape_1;
            float green_component = j_flt / shape_1;

            auto pose = ros_utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
            auto scale = ros_utils::createScale(0.03, 0.03, 0.03);
            auto color = ros_utils::createColor(0, green_component, blue_component, 1);
            auto marker = ros_utils::createMarker(marker_id_++, pose, scale, color, frame_id_);

            points_->markers.push_back(marker);
        }
    }

    return true;
}


} // namespace nano_mppic


#endif