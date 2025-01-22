#ifndef __NANO_MPPI_PATHDIST_CRITIC_HPP__
#define __NANO_MPPI_PATHDIST_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class PathDist : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::PathDistCritic cfg_;

    // FUNCTIONS

    public:
        PathDist();
        
        void configure(std::string name, 
                        nano_mppic::config::
                            PathDistCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        void setConfig(nano_mppic::config::PathDistCritic&);
};

PathDist::PathDist() { }

void PathDist::configure(std::string name, 
                        nano_mppic::config::
                            PathDistCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void PathDist::score(nano_mppic::objects::State& states,
                        nano_mppic::objects::Trajectory& trajectories,
                        nano_mppic::objects::Path& plan,
                        xt::xtensor<float,1>& costs,
                        bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
        std::cout << "NANO_MPPIC::PathDist critic not active\n";
        return;
    }

    if(nano_mppic::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        return;

    const size_t path_size = plan.x.shape(0) - 1;

    for(size_t i=0; i < trajectories.x.shape(2) /*time_steps*/; i+=cfg_.path_stride){

        size_t min_dist_path_point = nano_mppic::aux::findPathMinDistPoint(trajectories, plan, i);

        auto offseted_idx = std::min(min_dist_path_point, path_size);

        const auto path_x = plan.x(offseted_idx);
        const auto path_y = plan.y(offseted_idx);

        const auto x = xt::view(trajectories.x, xt::all(), i);
        const auto y = xt::view(trajectories.y, xt::all(), i);

        auto dists = xt::sqrt(
            xt::pow(x - path_x, 2) +
            xt::pow(y - path_y, 2));

        costs += xt::pow(cfg_.common.weight * std::move(dists), cfg_.common.power);
    }
    
}

void PathDist::setConfig(nano_mppic::config::PathDistCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
