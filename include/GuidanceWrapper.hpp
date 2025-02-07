#ifndef __NANO_MPPIC_GUIDANCE_WRAPPER_HPP__
#define __NANO_MPPIC_GUIDANCE_WRAPPER_HPP__

#include <guidance_planner/global_guidance.h>

#include "mppic.hpp"

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>

#include <vector>
#include <memory>

class GuidanceWrapper : public GuidancePlanner::GlobalGuidance {

    private:
        nano_mppic::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_ptr_{nullptr};

        std::vector<GuidancePlanner::Obstacle> obstacles_;
        std::vector<GuidancePlanner::Halfspace> static_obstacles_;

        bool isInCollision(unsigned char cost);

        void getCostMapObstacles();
        void getStaticObstacles();

        void resetObstacles();
        void resetStaticObstacles();

    public:
        GuidanceWrapper() = default;
        ~GuidanceWrapper();

        void configure(nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>&);

        void setGoals(nano_mppic::objects::Path&);
        void setReferencePlan(nano_mppic::objects::Path&);

        void setStart(nano_mppic::objects::Odometry2d odom);

        bool getPlan(nano_mppic::objects::Path&);
};

GuidanceWrapper::~GuidanceWrapper()
{
    delete costmap_ptr_;
}

void GuidanceWrapper::configure(nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros){
    costmap_ros_ptr_ = costmap_ros;
    costmap_ptr_ = costmap_ros->getCostmap();
}

bool GuidanceWrapper::getPlan(nano_mppic::objects::Path& plan)
{
    if(not costmap_ros_ptr_){
        std::cout << "GUIDANCE_WRAPPER::ERROR not configured!\n";
        return false;
    }

    // this->getStaticObstacles(); // probably only once ??

    this->getCostMapObstacles(); // retrieve obstacles from costmap (hardcore way)

    // Load the obstacles
    this->LoadObstacles(obstacles_, static_obstacles_);

    // Compute the guidance trajectories
    this->Update();

    // Show some results:
    if (this->Succeeded())
    {
        ROS_INFO_STREAM("Guidance planner found: " << this->NumberOfGuidanceTrajectories() << " trajectories");
        GuidancePlanner::CubicSpline3D &guidance_spline = this->GetGuidanceTrajectory(0).spline;
        ROS_INFO("[Best Trajectory]");

        auto guidance_trajectory = guidance_spline.GetTrajectory(); // Retrieves the trajectory: t -> (x, y))

        plan.reset(GuidancePlanner::Config::N); // resize output plan
        
        double t=0.0;
        for(int i=0; i < GuidancePlanner::Config::N; i++){
            Eigen::Vector2d pos = guidance_trajectory.getPoint(t);
            plan.x(i) = static_cast<float>(pos(0));
            plan.y(i) = static_cast<float>(pos(1));
            ROS_INFO_STREAM("\t[t = " << t << "]: (" << pos(0) << ", " << pos(1) << ")");
            t+=GuidancePlanner::Config::DT;
        }

        auto guidance_path = guidance_spline.GetPath(); // Retrieves the path: s -> (x, y)

        return true;
    }
    else
    {
        ROS_ERROR("\tGuidance planner found no trajectories that reach any of the goals!");
        return false;
    }

}

void GuidanceWrapper::setStart(nano_mppic::objects::Odometry2d odom)
{
    this->SetStart(Eigen::Vector2d(odom.x, odom.y), odom.yaw, odom.vx); // Position, yaw angle, velocity magnitude
}

void GuidanceWrapper::setGoals(nano_mppic::objects::Path& goals)
{
    if(goals.x.size() < 5){
        ROS_ERROR("Guidance planner: a minimum of 5 goals should be given to setGoals()");
        return;
    }

    // Using explicit goals as a set of 2D points with costs
    std::vector<GuidancePlanner::Goal> eigen_goals;
    eigen_goals.reserve(goals.x.size());
    for(int i=10; i < goals.x.size(); i+=5){
        eigen_goals.emplace_back(Eigen::Vector2d(goals.x(i), goals.y(i)), 1.); // cost = 1
    }
    this->SetGoals(eigen_goals);
}

void GuidanceWrapper::setReferencePlan(nano_mppic::objects::Path& plan)
{
    // Using a reference path in 2D as a RosTools::Spline2D object
    // Construct a spline in x, y
    std::vector<double> xx, yy;
    xx.reserve(plan.x.size()); 
    yy.reserve(plan.y.size());
    for(int i=0; i < plan.x.size(); i++){
        xx.push_back(plan.x(i));
        yy.push_back(plan.y(i));
    }
    std::shared_ptr<RosTools::Spline2D> reference_path = std::make_shared<RosTools::Spline2D>(xx, yy);

    double distance_on_spline = 0.; // Where we are on the spline
    double road_width = 6.;
    this->LoadReferencePath(distance_on_spline, reference_path, road_width);
}

void GuidanceWrapper::getCostMapObstacles()
{
    resetObstacles();
    for(unsigned int iy = 0; iy < costmap_ptr_->getSizeInCellsY(); iy++){
        for (unsigned int ix = 0; ix < costmap_ptr_->getSizeInCellsX(); ix++){

            unsigned char cost = costmap_ptr_->getCost(ix, iy);

            double wx, wy;
            costmap_ptr_->mapToWorld(ix, iy, wx, wy);

            if(this->isInCollision(cost)){
                obstacles_.emplace_back(0, Eigen::Vector2d(wx, wy), Eigen::Vector2d(0, 0), 
                                        GuidancePlanner::Config::DT, GuidancePlanner::Config::N, costmap_ptr_->getResolution());
                                    /* Obstacle ID, position, velocity in 2D, 
                                        integrator timestep, number of steps, radius */
            }
        }
    }
}

void GuidanceWrapper::getStaticObstacles() // TO-DO: define inequality via config (if necessary)
{
    resetStaticObstacles();
    /** @brief Static obstacles: Ax <= b */
    static_obstacles_.emplace_back(Eigen::Vector2d(0., 1.), 10.);  // y <= 10
    static_obstacles_.emplace_back(Eigen::Vector2d(0., -1.), 10.); // y >= -10
}

void GuidanceWrapper::resetObstacles()
{
    obstacles_.clear();
}

void GuidanceWrapper::resetStaticObstacles()
{
    static_obstacles_.clear();
}

bool GuidanceWrapper::isInCollision(unsigned char cost){
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

#endif