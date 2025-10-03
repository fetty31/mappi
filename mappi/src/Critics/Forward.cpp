/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Forward Critic. It prioritizes going forward.
 *
 * -----------------------------------------------------------------------------
 */

#include "Critics/Forward.hpp"

namespace mappi::critics {

Forward::Forward() { }

void Forward::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Forward::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& /*trajectories*/,
                    mappi::objects::Path& plan,
                    xt::xtensor<double,1>& costs,
                    bool & /*fail_flag*/)
{

    if(not costmap_ || not cfg_.common.active){
        return;
    }

    if(not mappi::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        return;

    using xt::evaluation_strategy::immediate;
    auto backward_motion = xt::maximum(-states.vx, 0);
    costs += xt::pow(
        xt::sum(
            std::move(backward_motion), {1}, immediate
            ) * cfg_.common.weight, cfg_.common.power);

}

void Forward::setConfig(mappi::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics