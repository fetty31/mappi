/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Goal Angle Critic. It penalizes trajectories that have heading values far from the goal heading.
 *
 * -----------------------------------------------------------------------------
 */

#include "Critics/GoalAngle.hpp"

namespace mappi::critics {

GoalAngle::GoalAngle() { }

void GoalAngle::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void GoalAngle::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<double,1>& costs,
                    bool & /*fail_flag*/)
{

    if(not costmap_ || not cfg_.common.active){
        return;
    }

    if(not mappi::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        return;

    const auto goal_idx = plan.x.shape(0) - 1;
    const double goal_yaw = plan.yaw(goal_idx);

    costs += xt::pow(
        xt::mean( xt::abs(
                    mappi::aux::shortest_angular_dist(trajectories.yaw, goal_yaw)
                    ), {1}) * cfg_.common.weight, cfg_.common.power);
}

void GoalAngle::setConfig(mappi::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics