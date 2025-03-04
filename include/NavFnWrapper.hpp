#ifndef __MAPPI_NAVFN_WRAPPER_HPP__
#define __MAPPI_NAVFN_WRAPPER_HPP__

#include <navfn/navfn.h>
#include <navfn/potarr_point.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <vector>
#include <string>
#include <mutex>
#include <memory>

class NavFnWrapper {

    public:
        NavFnWrapper();
        NavFnWrapper(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>&);
        ~NavFnWrapper();

        void configure(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>&);

        bool getPlan(mappi::objects::Odometry2d odom, 
                    mappi::objects::Path& goals,
                    mappi::objects::Path& out_plan);

        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

        bool computePotential(const geometry_msgs::Point& world_point);

        bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        double getPointPotential(const geometry_msgs::Point& world_point);

        bool validPointPotential(const geometry_msgs::Point& world_point);

        bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

        double getTolerance();
  
    protected:
        mappi::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_{nullptr};
        mappi::shared_ptr<navfn::NavFn> planner_;

        bool allow_unknown_;

    private:
        inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
            double dx = p1.pose.position.x - p2.pose.position.x;
            double dy = p1.pose.position.y - p2.pose.position.y;
            return dx*dx +dy*dy;
        }

        geometry_msgs::PoseStamped chooseGoal(mappi::objects::Path&);
        geometry_msgs::PoseStamped chooseStart(mappi::objects::Odometry2d&);
        void mapToWorld(double mx, double my, double& wx, double& wy);
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);

        double default_tolerance_;
        double start_shift_;
        std::mutex mutex_;

        std::string global_frame_;
};

NavFnWrapper::NavFnWrapper() 
        : allow_unknown_(true) {}

NavFnWrapper::NavFnWrapper(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros)
    : allow_unknown_(true) 
    {
    configure(name, costmap_ros);
}

NavFnWrapper::~NavFnWrapper() { delete costmap_; }

void NavFnWrapper::configure(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros)
{
    costmap_ros_ptr_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    planner_ = mappi::shared_ptr
                        <navfn::NavFn>(new navfn::NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()) );

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param<bool>("allow_unknown", allow_unknown_, true);
    private_nh.param<double>("default_tolerance", default_tolerance_, 0.0);
    private_nh.param<double>("start_shift", start_shift_, 0.0);
}

bool NavFnWrapper::getPlan(mappi::objects::Odometry2d odom, 
                            mappi::objects::Path& goals,
                            mappi::objects::Path& out_plan
                            )
{
    geometry_msgs::PoseStamped start;
    start = chooseStart(odom);

    geometry_msgs::PoseStamped goal;
    goal = chooseGoal(goals);

    std::vector<geometry_msgs::PoseStamped> plan_ros;
    if(this->makePlan(start, goal, plan_ros)){
        mappi::ros_utils::ros2mppic(plan_ros, out_plan);
        return true;
    }
    else
    {
        ROS_ERROR("NAVFN_WRAPPER::ERROR couldn't find any path!");
        return false;
    }
    
}

geometry_msgs::PoseStamped NavFnWrapper::chooseGoal(mappi::objects::Path& goals)
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

geometry_msgs::PoseStamped NavFnWrapper::chooseStart(mappi::objects::Odometry2d& odom)
{
    geometry_msgs::PoseStamped chosen_start;
    chosen_start.header.frame_id = global_frame_;
    chosen_start.pose.position.x = odom.x;
    chosen_start.pose.position.y = odom.y;
    
    float x, y; // point to start in base_link
    x = static_cast<float>(start_shift_); 
    y = 0.0f; 

    chosen_start.pose.position.x += x*cos(odom.yaw) + y*sin(odom.yaw);
    chosen_start.pose.position.y += -x*sin(odom.yaw) + y*cos(odom.yaw);

    return chosen_start;
}

bool NavFnWrapper::validPointPotential(const geometry_msgs::Point& world_point)
{
    return validPointPotential(world_point, default_tolerance_);
}

bool NavFnWrapper::validPointPotential(const geometry_msgs::Point& world_point, double tolerance)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    double resolution = costmap_->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
        p.x = world_point.x - tolerance;
        while(p.x <= world_point.x + tolerance){
            double potential = getPointPotential(p);
            if(potential < /*navfn::*/POT_HIGH){
            return true;
            }
            p.x += resolution;
        }
        p.y += resolution;
    }

    return false;
}

double NavFnWrapper::getPointPotential(const geometry_msgs::Point& world_point)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
        return std::numeric_limits<double>::max();

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
}

bool NavFnWrapper::computePotential(const geometry_msgs::Point& world_point)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
        return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
}

void NavFnWrapper::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void NavFnWrapper::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

bool NavFnWrapper::makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

bool NavFnWrapper::makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::unique_lock<std::mutex> guard(mutex_);
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if(start.header.frame_id != global_frame_){
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
        ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, mx, my);

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap_->worldToMap(wx, wy, mx, my)){
        if(tolerance <= 0.0){
            ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }
        mx = 0;
        my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    //bool success = planner_->calcNavFnAstar();
    planner_->calcNavFnDijkstra(true);

    double resolution = costmap_->getResolution();
    geometry_msgs::PoseStamped p, best_pose;
    p = goal;

    bool found_legal = false;
    double best_sdist = std::numeric_limits<double>::max();

    p.pose.position.y = goal.pose.position.y - tolerance;

    while(p.pose.position.y <= goal.pose.position.y + tolerance){
        p.pose.position.x = goal.pose.position.x - tolerance;
        while(p.pose.position.x <= goal.pose.position.x + tolerance){
            double potential = getPointPotential(p.pose.position);
            double sdist = sq_distance(p, goal);
            if(potential < /*navfn::*/POT_HIGH && sdist < best_sdist){
            best_sdist = sdist;
            best_pose = p;
            found_legal = true;
            }
            p.pose.position.x += resolution;
        }
        p.pose.position.y += resolution;
    }

    if(found_legal){
        //extract the plan
        if(getPlanFromPotential(best_pose, plan)){
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = best_pose;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        }
        else{
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }

    return !plan.empty();
}

bool NavFnWrapper::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    //the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();

    for(int i = len - 1; i >= 0; --i){
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(x[i], y[i], world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }

    return !plan.empty();
}

double NavFnWrapper::getTolerance()
{
    return default_tolerance_;
}

#endif