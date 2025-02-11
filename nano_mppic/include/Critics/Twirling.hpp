#ifndef __NANO_MPPI_TWIRLING_CRITIC_HPP__
#define __NANO_MPPI_TWIRLING_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class Twirling : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        Twirling();
        
        void configure(std::string name, 
                        nano_mppic::config::
                            GenericCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        void setConfig(nano_mppic::config::GenericCritic&);
};

Twirling::Twirling() { }

void Twirling::configure(std::string name, 
                    nano_mppic::config::
                        GenericCritic& config,
                    nano_mppic::shared_ptr
                        <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Twirling::score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
        return;
    }

    using xt::evaluation_strategy::immediate;
    
    const auto wz = xt::abs(states.wz);
    costs += xt::pow(xt::mean(wz, {1}, immediate) * cfg_.common.weight, cfg_.common.power);

}

void Twirling::setConfig(nano_mppic::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
