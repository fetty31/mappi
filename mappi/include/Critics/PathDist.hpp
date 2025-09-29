/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Path Dist Critic. It penalizes trajectories comparing its distance w.r.t. the reference path.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_PATHDIST_CRITIC_HPP__
#define __MAPPI_PATHDIST_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class PathDist : public Critic {

    // VARIABLES

    protected:
        mappi::config::PathDistCritic cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new PathDist critic
         * 
         */
        PathDist();

         /**
         * @brief Configure the Critic
         * 
         * @param name Name of the critic
         * @param config Configuration object
         * @param costmap_ros Costmap 2D object
         */
        void configure(std::string name, 
                        mappi::config::
                            PathDistCritic& config,
                        mappi::shared_ptr
                            <mappi::utils::CostmapInterface>& costmap_ros);

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
                    xt::xtensor<double,1>& costs,
                    bool &fail_flag) override;

        /**
         * @brief Overwrite the current configuration
         * 
         * @param config New configuration
         */
        void setConfig(mappi::config::PathDistCritic&);
};

PathDist::PathDist() { }

void PathDist::configure(std::string name, 
                        mappi::config::
                            PathDistCritic& config,
                        mappi::shared_ptr
                            <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void PathDist::score(mappi::objects::State& states,
                        mappi::objects::Trajectory& trajectories,
                        mappi::objects::Path& plan,
                        xt::xtensor<double,1>& costs,
                        bool & /*fail_flag*/)
{

    if(not costmap_ || not cfg_.common.active){
        return;
    }

    if(mappi::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
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
    auto && cost = xt::xtensor<double, 1>::from_shape({costs.shape(0)});

    for (size_t t = 0; t < batch_size; ++t) {
        double summed_dist = 0;
        for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
            double min_dist_sq = std::numeric_limits<double>::max();
            size_t min_s = 0;

            // Find closest path segment to the trajectory point
            for (size_t s = 0; s < path_segments_count - 1; s++) {
                double dx = P_x(s) - T_x(t, p);
                double dy = P_y(s) - T_y(t, p);
                double dist_sq = dx * dx + dy * dy;
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

void PathDist::setConfig(mappi::config::PathDistCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics


#endif
