#ifndef __MAPPI_PATHFOLLOW_CRITIC_HPP__
#define __MAPPI_PATHFOLLOW_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class PathFollow : public Critic {

    // VARIABLES

    protected:
        mappi::config::PathCritic cfg_;

    // FUNCTIONS

    public:
        PathFollow();
        
        void configure(std::string name, 
                        mappi::config::
                            PathCritic& config,
                        mappi::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        void setConfig(mappi::config::PathCritic&);
};

PathFollow::PathFollow() { }

void PathFollow::configure(std::string name, 
                        mappi::config::
                            PathCritic& config,
                        mappi::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void PathFollow::score(mappi::objects::State& states,
                        mappi::objects::Trajectory& trajectories,
                        mappi::objects::Path& plan,
                        xt::xtensor<float,1>& costs,
                        bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
        return;
    }

    if(mappi::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        return;

    const size_t path_size = plan.x.shape(0) - 1;

    size_t min_dist_path_point = mappi::aux::findPathMinDistPoint(trajectories, plan);

    auto offseted_idx = std::min(min_dist_path_point + cfg_.offset_from_furthest, path_size);

    bool valid = false;
    while ( (not valid) && (offseted_idx < path_size-1) ) {
        valid = plan.free(offseted_idx);
        if (not valid) offseted_idx++;
    }

    const auto path_x = plan.x(offseted_idx);
    const auto path_y = plan.y(offseted_idx);

    const auto last_x = xt::view(trajectories.x, xt::all(), -1);
    const auto last_y = xt::view(trajectories.y, xt::all(), -1);

    auto dists = xt::sqrt(
        xt::pow(last_x - path_x, 2) +
        xt::pow(last_y - path_y, 2));

    costs += xt::pow(cfg_.common.weight * std::move(dists), cfg_.common.power);

}

void PathFollow::setConfig(mappi::config::PathCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics


#endif
