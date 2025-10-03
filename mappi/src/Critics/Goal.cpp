/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Goal Critic. It penalizes trajectories far from the goal.
 *
 * -----------------------------------------------------------------------------
 */

#include "Critics/Goal.hpp"

namespace mappi::critics {

Goal::Goal() { }

void Goal::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Goal::score(mappi::objects::State& states,
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

    const auto goal_x = plan.x(goal_idx);
    const auto goal_y = plan.y(goal_idx);

    const auto last_x = xt::view(trajectories.x, xt::all(), -1);
    const auto last_y = xt::view(trajectories.y, xt::all(), -1);

    auto dists = xt::sqrt(
        xt::pow(last_x - goal_x, 2) +
        xt::pow(last_y - goal_y, 2));

    costs += xt::pow(std::move(dists) * cfg_.common.weight, cfg_.common.power);

}

void Goal::setConfig(mappi::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics