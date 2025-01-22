#ifndef __NANO_MPPI_PATHFOLLOW_CRITIC_HPP__
#define __NANO_MPPI_PATHFOLLOW_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class PathFollow : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::PathFollowCritic cfg_;

    // FUNCTIONS

    public:
        PathFollow();
        
        void configure(std::string name, 
                        nano_mppic::config::
                            PathFollowCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        void setConfig(nano_mppic::config::PathFollowCritic&);
};

PathFollow::PathFollow() { }

void PathFollow::configure(std::string name, 
                        nano_mppic::config::
                            PathFollowCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void PathFollow::score(nano_mppic::objects::State& states,
                        nano_mppic::objects::Trajectory& trajectories,
                        nano_mppic::objects::Path& plan,
                        xt::xtensor<float,1>& costs,
                        bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
        std::cout << "NANO_MPPIC::PathFollow critic not active\n";
        return;
    }

    if(nano_mppic::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        return;

    const size_t path_size = plan.x.shape(0) - 1;

    size_t min_dist_path_point = nano_mppic::aux::findPathMinDistPoint(trajectories, plan);

    auto offseted_idx = std::min(min_dist_path_point + cfg_.offset_from_furthest, path_size);

    /*To-Do:
        - check if offseted_idx is a valid point 
    */

    // bool valid = false;
    // while (!valid && offseted_idx < path_size - 1) {
    //     valid = (*data.path_pts_valid)[offseted_idx];
    //     if (!valid) {
    //     offseted_idx++;
    //     }
    // }

    const auto path_x = plan.x(offseted_idx);
    const auto path_y = plan.y(offseted_idx);

    const auto last_x = xt::view(trajectories.x, xt::all(), -1);
    const auto last_y = xt::view(trajectories.y, xt::all(), -1);

    auto dists = xt::sqrt(
        xt::pow(last_x - path_x, 2) +
        xt::pow(last_y - path_y, 2));

    costs += xt::pow(cfg_.common.weight * std::move(dists), cfg_.common.power);
    
}

void PathFollow::setConfig(nano_mppic::config::PathFollowCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
