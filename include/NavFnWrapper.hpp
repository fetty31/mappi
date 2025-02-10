#ifndef __NANO_MPPIC_NAVFN_WRAPPER_HPP__
#define __NANO_MPPIC_NAVFN_WRAPPER_HPP__

#include <navfn/navfn_ros.h>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <vector>
#include <string>
#include <memory>

class NavFnWrapper : public navfn::NavfnROS {

    private:
        std::string global_frame_;

        nano_mppic::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;

        geometry_msgs::PoseStamped chooseGoal(nano_mppic::objects::Path&);

    public:
        NavFnWrapper() = default;
        ~NavFnWrapper() = default;

        void configure(std::string name, nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>&);

        bool getPlan(nano_mppic::objects::Odometry2d odom, 
                    nano_mppic::objects::Path& goals,
                    nano_mppic::objects::Path& out_plan);
};

void NavFnWrapper::configure(std::string name, nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros){
    global_frame_ = costmap_ros->getGlobalFrameID();
    costmap_ros_ptr_ = costmap_ros;
    this->initialize(name, costmap_ros->getCostmap(), global_frame_);
}

bool NavFnWrapper::getPlan(nano_mppic::objects::Odometry2d odom, 
                            nano_mppic::objects::Path& goals,
                            nano_mppic::objects::Path& out_plan
                            )
{
    geometry_msgs::PoseStamped start;
    start.header.frame_id = global_frame_;
    start.pose.position.x = odom.x;
    start.pose.position.y = odom.y;

    geometry_msgs::PoseStamped goal;
    goal = chooseGoal(goals);

    std::vector<geometry_msgs::PoseStamped> plan_ros;
    if(this->makePlan(start, goal, plan_ros)){
        nano_mppic::ros_utils::ros2mppic(plan_ros, out_plan);
        return true;
    }
    else
    {
        ROS_ERROR("NAVFN_WRAPPER::ERROR couldn't find any path!");
        return false;
    }
    
}

geometry_msgs::PoseStamped NavFnWrapper::chooseGoal(nano_mppic::objects::Path& goals)
{
    geometry_msgs::PoseStamped chosen_goal;
    chosen_goal.header.frame_id = global_frame_;
    
    size_t N = goals.x.size()-1;
    unsigned int mx, my;

    // Search goals vector until the chosen goal is inside the local costmap
    while( (N > 0) && 
        (not costmap_ros_ptr_->getCostmap()->worldToMap(goals.x(N), goals.y(N), mx, my)) 
    ){
        N--;
    }

    chosen_goal.pose.position.x = goals.x(N);
    chosen_goal.pose.position.y = goals.y(N);

    return chosen_goal;
}

#endif
