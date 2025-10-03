/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Twirling Critic. It penalizes angular velocity (or steering rate) values far from its mean.
 *
 * -----------------------------------------------------------------------------
 */

#include "Critics/Twirling.hpp"

namespace mappi::critics {

Twirling::Twirling() { }

void Twirling::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Twirling::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& /*trajectories*/,
                    mappi::objects::Path& /*plan*/,
                    xt::xtensor<double,1>& costs,
                    bool & /*fail_flag*/)
{

    if(not costmap_ || not cfg_.common.active){
        return;
    }

    using xt::evaluation_strategy::immediate;

    // xt::stddev possibly introduces overhead (https://github.com/xtensor-stack/xtensor/issues/1826)

    // costs += xt::pow( xt::variance(states.wz, {1}, 0/*ddof*/, immediate) * cfg_.common.weight, cfg_.common.power);
    
    const auto wz = xt::abs(states.wz);
    costs += xt::pow(xt::mean(wz, {1}, immediate) * cfg_.common.weight, cfg_.common.power);
}

void Twirling::setConfig(mappi::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics