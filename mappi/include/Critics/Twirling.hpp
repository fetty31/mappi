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

#ifndef __MAPPI_TWIRLING_CRITIC_HPP__
#define __MAPPI_TWIRLING_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class Twirling : public Critic {

    // VARIABLES

    protected:
        mappi::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Twirling critic
         * 
         */
        Twirling();

         /**
         * @brief Configure the Critic
         * 
         * @param name Name of the critic
         * @param config Configuration object
         * @param costmap_ros Costmap 2D object
         */
        void configure(std::string name, 
                        mappi::config::
                            GenericCritic& config,
                        mappi::shared_ptr
                            <nav2_costmap_2d::Costmap2DROS>& costmap_ros);

        /**
         * @brief Score the sampled trajectories
         * 
         * @param states Current states
         * @param trajectories Sampled trajectories
         * @param plan Reference path
         * @param costs Output cost for each trajectory
         * @param fail_flag Output fail flag. True if all trajectories collide (or are unfeasible) 
         */
        void score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        /**
         * @brief Overwrite the current configuration
         * 
         * @param config New configuration
         */
        void setConfig(mappi::config::GenericCritic&);
};

Twirling::Twirling() { }

void Twirling::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <nav2_costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Twirling::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
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


#endif
