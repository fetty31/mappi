#ifndef __MAPPI_NAVFN_WRAPPER_HPP__
#define __MAPPI_NAVFN_WRAPPER_HPP__

#include <navfn/navfn.h>
#include <navfn/potarr_point.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <costmap_2d/costmap_2d_publisher.h>

#include "mppic.hpp"
#include "ROSutils.hpp"

#include <vector>
#include <string>
#include <mutex>
#include <memory>

class NavFnWrapper {

    public:
        /**
         * @brief Construct a new Nav Fn Wrapper object
         * 
         */
        NavFnWrapper();

        /**
         * @brief Construct a new Nav Fn Wrapper object
         * 
         * @param name 
         * @param costmap_ros 
         */
        NavFnWrapper(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>&);

        /**
         * @brief Destroy the Nav Fn Wrapper object
         * 
         */
        ~NavFnWrapper();

        /**
         * @brief Configure Nav Fn Wrapper object
         * 
         * @param name 
         * @param costmap_ros 
         */
        void configure(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>&);

        /**
         * @brief Get the output plan
         * 
         * @param odom Current robot odometry
         * @param goals High level plan (global plan)
         * @param out_plan Output plan
         * @return true 
         * @return false 
         */
        bool getPlan(mappi::objects::Odometry2d odom, 
                    mappi::objects::Path& goals,
                    mappi::objects::Path& out_plan);
        
        /**
         * @brief Compute new plan
         * 
         * @param start 
         * @param goal 
         * @param plan 
         * @return true 
         * @return false 
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);
        
        /**
         * @brief Compute new plan with given goal tolerance
         * 
         * @param start 
         * @param goal 
         * @param tolerance 
         * @param plan 
         * @return true 
         * @return false 
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);
        
        /**
         * @brief Compute point potential
         * 
         * @param world_point 
         * @return true 
         * @return false 
         */
        bool computePotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Get the Plan From Potential object
         * 
         * @param goal 
         * @param plan 
         * @return true 
         * @return false 
         */
        bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Get the Point Potential object
         * 
         * @param world_point 
         * @return double 
         */
        double getPointPotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Check if potential is valid
         * 
         * @param world_point 
         * @return true 
         * @return false 
         */
        bool validPointPotential(const geometry_msgs::Point& world_point);
        
        /**
         * @brief Check if potential is valid for given tolerance
         * 
         * @param world_point 
         * @param tolerance 
         * @return true 
         * @return false 
         */
        bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);
        
        /**
         * @brief Get the current tolerance
         * 
         * @return double 
         */
        double getTolerance();
        
        /**
         * @brief Check if we should replan or not
         * 
         * @param plan 
         * @param goal 
         * @return true 
         * @return false 
         */
        bool timeToPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                        geometry_msgs::PoseStamped& goal);
  
    protected:
        mappi::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_{nullptr};
        mappi::shared_ptr<navfn::NavFn> planner_;

        bool allow_unknown_;

        std::vector<geometry_msgs::PoseStamped> plan_ros_;
        geometry_msgs::PoseStamped last_goal_;

        costmap_2d::Costmap2DPublisher* costmap_pub_;

        ros::Publisher goal_pub_;
        ros::Publisher start_pub_;

    private:
        /**
         * @brief Compute squared distance
         * 
         * @param p1 
         * @param p2 
         * @return double 
         */
        inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
            double dx = p1.pose.position.x - p2.pose.position.x;
            double dy = p1.pose.position.y - p2.pose.position.y;
            return dx*dx +dy*dy;
        }

        /**
         * @brief Choose a goal from the global plan
         * 
         * @param goals Global plan
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped chooseGoal(mappi::objects::Path&);

        /**
         * @brief Choose a goal from the global plan by distance
         * 
         * @param goals Global plan
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped chooseGoalByDistance(mappi::objects::Path&);

        /**
         * @brief Choose starting point
         * 
         * @param mode 
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped chooseStart(mappi::objects::Odometry2d&, int mode=0);

        /**
         * @brief Fill robot's footprint
         * 
         * @param odom 
         */
        void fillRobotFootprint(mappi::objects::Odometry2d& odom);

        /**
         * @brief Transform from costmap index to world coordinates
         * 
         * @param mx 
         * @param my 
         * @param wx 
         * @param wy 
         */
        void mapToWorld(double mx, double my, double& wx, double& wy);

        /**
         * @brief Clear robot cell from costmap
         * 
         * @param mx 
         * @param my 
         */
        void clearRobotCell(unsigned int mx, unsigned int my);

        /**
         * @brief Fill robot cell from costmap
         * 
         * @param mx 
         * @param my 
         */
        void fillRobotCell(unsigned int mx, unsigned int my);

        /**
         * @brief Check if given cost is in collision
         * 
         * @param cost 
         * @return true 
         * @return false 
         */
        bool isInCollision(unsigned char cost);

        /**
         * @brief Publish start and goal points
         * 
         * @param start 
         * @param goal 
         */
        void publishStartAndGoal(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);

        double default_tolerance_;
        double start_shift_;
        float lookahead_dist_;
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
    costmap_ = new costmap_2d::Costmap2D(); // allocate memory

    costmap_ros_ptr_ = costmap_ros;
    global_frame_ = costmap_ros->getGlobalFrameID();
    planner_ = mappi::shared_ptr
                        <navfn::NavFn>(new navfn::NavFn(costmap_ros->getCostmap()->getSizeInCellsX(), 
                                                        costmap_ros->getCostmap()->getSizeInCellsY()) 
                                        );

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param<bool>("allow_unknown", allow_unknown_, true);
    private_nh.param<double>("default_tolerance", default_tolerance_, 0.0);
    private_nh.param<double>("start_shift", start_shift_, 0.0);
    private_nh.param<float>("lookahead_dist", lookahead_dist_, 0.0f);

    costmap_pub_ = new costmap_2d::Costmap2DPublisher(&private_nh, costmap_, global_frame_,
                                                        "costmap", true);

    goal_pub_ = private_nh.advertise<visualization_msgs::Marker>("goal", 1);
    start_pub_ = private_nh.advertise<visualization_msgs::Marker>("start", 1);

}

bool NavFnWrapper::getPlan(mappi::objects::Odometry2d odom, 
                            mappi::objects::Path& goals,
                            mappi::objects::Path& out_plan
                            )
{
    std::cout << "NAVFN_WRAPPER:: computing new plan...\n";

    costmap_2d::Costmap2D costmap_cpy = *costmap_ros_ptr_->getCostmap(); // define cpy of costmap (because we'll fill some cells before planning)
    *costmap_ = costmap_cpy;

    geometry_msgs::PoseStamped start;
    start = chooseStart(odom, 1); // mode=1 (shift start point along positive x-axis)

    geometry_msgs::PoseStamped goal;
    goal = (lookahead_dist_ > 0) ? chooseGoalByDistance(goals) : chooseGoal(goals);

    // Publish chosen start & goal (debug)
    this->publishStartAndGoal(start, goal);

    // Decide if we really need to re-compute trajectory
    if(not timeToPlan(plan_ros_, goal)){
        mappi::ros_utils::ros2mppic(plan_ros_, out_plan);
        ROS_WARN("NAVFN_WRAPPER:: reusing last plan");
        return true; // sending last computed plan
    }
    last_goal_ = goal;

    unsigned int mx, my;
    if(!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)){
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

    // Clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(mx, my);

    // (optional) Add virtual costs around the robot (Ona needs to avoid perpendicular trajectories)
    fillRobotFootprint(odom);

    if(this->makePlan(start, goal, plan_ros_)){
        mappi::ros_utils::ros2mppic(plan_ros_, out_plan);
        return true;
    }
    else if(this->makePlan(chooseStart(odom, 2), goal, plan_ros_)){ // mode=2 (shift start point along negative x-axis)
        ROS_WARN("NAVFN_WRAPPER:: path found with start point shifting along negative x-axis!");
        mappi::ros_utils::ros2mppic(plan_ros_, out_plan);
        return true;
    }
    else if(this->makePlan(chooseStart(odom), goal, plan_ros_)){ // mode=0 (do not shift start point)
        ROS_WARN("NAVFN_WRAPPER:: path found with no start point shifting!");
        mappi::ros_utils::ros2mppic(plan_ros_, out_plan);
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
    while( (N > 1) && 
        (not costmap_->worldToMap(goals.x(N), goals.y(N), mx, my)) 
    ){
        N--;
    }

    // Check if chosen goal is in free space
    if(not this->isInCollision(costmap_->getCost(mx, my))){
        chosen_goal.pose.position.x = goals.x(N);
        chosen_goal.pose.position.y = goals.y(N);
        return chosen_goal;
    }

    // If goal is in collision, create perpendicular virtual goals
    std::vector<float> p_points = mappi::aux::linspace(0.0f, 4.0f, 15); // points to check (perpendicular to the path)

    float yaw = std::atan2( goals.y(N)-goals.y(N-1), goals.x(N)-goals.x(N-1) );
    float xN = goals.x(N);
    float yN = goals.y(N);

    float x,y;
    bool found = false;
    for(int i=0; i < p_points.size(); i++){
        x = xN - p_points[i]*sin(yaw);
        y = yN + p_points[i]*cos(yaw);
        
        costmap_->worldToMap(x, y, mx, my);
        if(not this->isInCollision(costmap_->getCost(mx, my))){
            found = true;
            break;
        }

        x = xN + p_points[i]*sin(yaw);
        y = yN - p_points[i]*cos(yaw);
        
        costmap_->worldToMap(x, y, mx, my);
        if(not this->isInCollision(costmap_->getCost(mx, my))){
            found = true;
            break;
        }
    }

    if(found){
        chosen_goal.pose.position.x = x;
        chosen_goal.pose.position.y = y;
    }else{
        chosen_goal.pose.position.x = goals.x(N);
        chosen_goal.pose.position.y = goals.y(N);
    }

    return chosen_goal;
}

geometry_msgs::PoseStamped NavFnWrapper::chooseGoalByDistance(mappi::objects::Path& goals)
{
    geometry_msgs::PoseStamped chosen_goal;
    chosen_goal.header.frame_id = global_frame_;
    
    size_t N = mappi::aux::getIdxFromDistance(goals, lookahead_dist_);
    unsigned int mx, my;

    // Search goals vector until the chosen goal is inside the local costmap
    while( (N > 0) && 
        (not costmap_->worldToMap(goals.x(N), goals.y(N), mx, my)) 
    ){
        N--;
    }

    // Check if chosen goal is in free space
    if(not this->isInCollision(costmap_->getCost(mx, my))){
        chosen_goal.pose.position.x = goals.x(N);
        chosen_goal.pose.position.y = goals.y(N);
        return chosen_goal;
    }

    // If goal is in collision, create perpendicular virtual goals
    std::vector<float> p_points = mappi::aux::linspace(0.0f, 4.0f, 15); // points to check (perpendicular to the path)

    float yaw = std::atan2( goals.y(N)-goals.y(N-1), goals.x(N)-goals.x(N-1) );
    float xN = goals.x(N);
    float yN = goals.y(N);

    float x,y;
    bool found = false;
    for(int i=0; i < p_points.size(); i++){
        x = xN - p_points[i]*sin(yaw);
        y = yN + p_points[i]*cos(yaw);
        
        costmap_->worldToMap(x, y, mx, my);
        if(not this->isInCollision(costmap_->getCost(mx, my))){
            found = true;
            break;
        }

        x = xN + p_points[i]*sin(yaw);
        y = yN - p_points[i]*cos(yaw);
        
        costmap_->worldToMap(x, y, mx, my);
        if(not this->isInCollision(costmap_->getCost(mx, my))){
            found = true;
            break;
        }
    }

    if(found){
        chosen_goal.pose.position.x = x;
        chosen_goal.pose.position.y = y;
    }else{
        chosen_goal.pose.position.x = goals.x(N);
        chosen_goal.pose.position.y = goals.y(N);
    }

    return chosen_goal;
}

geometry_msgs::PoseStamped NavFnWrapper::chooseStart(mappi::objects::Odometry2d& odom, int mode)
{
    geometry_msgs::PoseStamped chosen_start;
    chosen_start.header.frame_id = global_frame_;
    chosen_start.pose.position.x = odom.x;
    chosen_start.pose.position.y = odom.y;
    
    float x, y; // point to start in base_link
    y = 0.0f; 

    if(mode == 1)       // shift start point along positive x-axis
        x = static_cast<float>(start_shift_); 
    else if(mode == 2)  // shift start point along negative x-axis
        x = -static_cast<float>(start_shift_); 
    else                // no shifting
        x = 0.0f;

    chosen_start.pose.position.x += x*cos(odom.yaw) - y*sin(odom.yaw);
    chosen_start.pose.position.y += x*sin(odom.yaw) + y*cos(odom.yaw);

    return chosen_start;
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
    if(!costmap_->worldToMap(wx, wy, mx, my))
        return false;

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
            ROS_WARN("The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
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
        ROS_WARN("The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
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

void NavFnWrapper::fillRobotFootprint(mappi::objects::Odometry2d& odom)
{
    std::vector<geometry_msgs::Point> footprint = costmap_ros_ptr_->getRobotFootprint();
    // NOTE: here we are assuming the footprint is ordered as in footprint param
    //      - in Ona's case: [FrontLeft, FrontRight, RearRight, RearLeft] (see vaive_rosnav/config/costmap_common_params.yaml)

    if( footprint.size() < 4 ){
        ROS_WARN("NAVFN_WRAPPER::Robot's footprint is not a rectangle, no virtual costs will be added");
        return;
    }

    ROS_INFO("NAVFN_WRAPPER:: filling virtual costs");

    // fill robot's laterals with virtual costs
    unsigned int mx, my;
    double dx = costmap_->getResolution()/2.0;
    double x, y;

    double extra_space = 1.2;

    // get number of cells on the laterals
    int n_cells = std::ceil( (std::fabs(footprint[0].x) + std::fabs(footprint[2].x) + 2*extra_space)/dx );

    // left side
    y = footprint[0].y; // FrontLeft y coord
    x = footprint[0].x + extra_space; // FrontLeft x coord
    for(int i=0; i < n_cells; i++){
        x -= dx;
        double x_odom = odom.x + x*cos(odom.yaw) - y*sin(odom.yaw);
        double y_odom = odom.y + x*sin(odom.yaw) + y*cos(odom.yaw);

        if(costmap_->worldToMap(x_odom, y_odom, mx, my))
            fillRobotCell(mx, my);
    }

    // right side
    y = footprint[1].y; // FrontRight y coord
    x = footprint[1].x + extra_space; // FrontRight x coord
    for(int i=0; i < n_cells; i++){
        x -= dx;
        double x_odom = odom.x + x*cos(odom.yaw) - y*sin(odom.yaw);
        double y_odom = odom.y + x*sin(odom.yaw) + y*cos(odom.yaw);

        if(costmap_->worldToMap(x_odom, y_odom, mx, my))
            fillRobotCell(mx, my);
    }

}

void NavFnWrapper::clearRobotCell(unsigned int mx, unsigned int my)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void NavFnWrapper::fillRobotCell(unsigned int mx, unsigned int my)
{
    if(not costmap_ros_ptr_){
        ROS_ERROR("NAVFN_WRAPPER::This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be lethal
    costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
}

void NavFnWrapper::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

double NavFnWrapper::getTolerance()
{
    return default_tolerance_;
}

bool NavFnWrapper::timeToPlan(std::vector<geometry_msgs::PoseStamped>& plan, 
                                geometry_msgs::PoseStamped& goal )
{
    if(plan.size() < 1)
        return true;

    if( sqrt(sq_distance(goal, last_goal_)) >  0.1 )
    {
        ROS_ERROR("NAVFN_WRAPPER:: new goal received");
        return true;
    }

    unsigned int mx, my;
    for(auto point : plan){
        if(not costmap_->worldToMap(point.pose.position.x, point.pose.position.y, mx, my))
            return true;
        if(isInCollision(costmap_->getCost(mx, my)))
            return true; 
    }

    return false; // do not re-compute trajectory
}

bool NavFnWrapper::isInCollision(unsigned char cost)
{
    bool is_tracking_unkown = 
        costmap_ros_ptr_->getLayeredCostmap()->isTrackingUnknown();

    switch(cost) {
        case(costmap_2d::LETHAL_OBSTACLE):
            return true;
        case(costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
            return true;
        case(costmap_2d::NO_INFORMATION):
            return is_tracking_unkown ? false : true;
        case(costmap_2d::FREE_SPACE):
            return false;
        default:
            return false;
    }
}

void NavFnWrapper::publishStartAndGoal(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    auto goal_pose  = mappi::ros_utils::createPose(goal.pose.position.x, goal.pose.position.y, 0.1);
    auto start_pose = mappi::ros_utils::createPose(start.pose.position.x, start.pose.position.y, 0.1);

    auto scale = mappi::ros_utils::createScale(0.5, 0.5, 0.5);

    auto goal_color  = mappi::ros_utils::createColor(1, 0, 0, 1);   // red
    auto start_color = mappi::ros_utils::createColor(0, 0, 1, 1);  // blue

    auto goal_m  = mappi::ros_utils::createMarker(0, goal_pose, scale, goal_color, global_frame_, "navfn");
    auto start_m = mappi::ros_utils::createMarker(1, start_pose, scale, start_color, global_frame_, "navfn");

    goal_pub_.publish(goal_m);
    start_pub_.publish(start_m);
}

#endif