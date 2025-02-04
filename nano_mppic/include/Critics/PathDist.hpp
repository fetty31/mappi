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

    const auto & T_x = trajectories.x;
    const auto & T_y = trajectories.y;

    const auto P_x = xt::view(plan.x, xt::range(xt::placeholders::_, -1));  // path points
    const auto P_y = xt::view(plan.y, xt::range(xt::placeholders::_, -1));  // path points

    int trajectory_point_step_ = cfg_.traj_stride;

    const size_t batch_size = T_x.shape(0);
    const size_t time_steps = T_x.shape(1);
    const size_t traj_pts_eval = floor(time_steps / trajectory_point_step_);
    const size_t path_segments_count = plan.x.shape(0) - 1;
    auto && cost = xt::xtensor<float, 1>::from_shape({costs.shape(0)});

    for (size_t t = 0; t < batch_size; ++t) {
        float summed_dist = 0;
        for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
            double min_dist_sq = std::numeric_limits<float>::max();
            size_t min_s = 0;

            // Find closest path segment to the trajectory point
            for (size_t s = 0; s < path_segments_count - 1; s++) {
                float dx = P_x(s) - T_x(t, p);
                float dy = P_y(s) - T_y(t, p);
                float dist_sq = dx * dx + dy * dy;
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    min_s = s;
                }
            }

            // The nearest path point needs to be not in collision, else
            // let the obstacle critic take over in this region due to dynamic obstacles
            if( (min_s != 0) && plan.free(min_s))
                summed_dist += std::sqrt(min_dist_sq);
        }

        cost[t] = summed_dist / traj_pts_eval;
    }

    costs += xt::pow(std::move(cost) * cfg_.common.weight, cfg_.common.power);
    
}

void PathDist::setConfig(nano_mppic::config::PathDistCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
