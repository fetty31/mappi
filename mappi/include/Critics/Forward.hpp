#ifndef __MAPPI_FORWARD_CRITIC_HPP__
#define __MAPPI_FORWARD_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class Forward : public Critic {

    // VARIABLES

    protected:
        mappi::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        Forward();
        
        void configure(std::string name, 
                        mappi::config::
                            GenericCritic& config,
                        mappi::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        void setConfig(mappi::config::GenericCritic&);
};

Forward::Forward() { }

void Forward::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Forward::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
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


#endif
