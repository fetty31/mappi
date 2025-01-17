#ifndef __NANO_MPPI_GOAL_CRITIC_HPP__
#define __NANO_MPPI_GOAL_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class Goal : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        Goal();
        
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

Goal::Goal() { }

void Goal::configure(std::string name, 
                    nano_mppic::config::
                        GenericCritic& config,
                    nano_mppic::shared_ptr
                        <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Goal::score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag)
{

    std::cout << "NANO_MPPIC::MPPIc::Goal::score()\n";

    if(not costmap_ros_ptr_){
        std::cout << "NANO_MPPIC::GOAL Error: no costmap object passed to critic function!\n";
        return;
    }

    /*To-Do:
        - check if we are closer than cfg_.common.threshold_ to the trajectory
            . if closer -> do not compute any cost (return)
            . else -> pass
    */

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

void Goal::setConfig(nano_mppic::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
