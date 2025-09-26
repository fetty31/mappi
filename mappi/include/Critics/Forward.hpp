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
        /**
         * @brief Construct a new Forward critic
         * 
         */
        Forward();
        
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

Forward::Forward() { }

void Forward::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <nav2_costmap_2d::Costmap2DROS>& costmap_ros){

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
