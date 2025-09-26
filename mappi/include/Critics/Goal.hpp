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

#ifndef __MAPPI_GOAL_CRITIC_HPP__
#define __MAPPI_GOAL_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class Goal : public Critic {

    // VARIABLES

    protected:
        mappi::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Goal critic
         * 
         */
        Goal();

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

Goal::Goal() { }

void Goal::configure(std::string name, 
                    mappi::config::
                        GenericCritic& config,
                    mappi::shared_ptr
                        <nav2_costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Goal::score(mappi::objects::State& states,
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


#endif
