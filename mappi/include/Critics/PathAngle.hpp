/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Path Angle Critic. It penalizes trajectories comparing its heading angles with the
 *   reference path heading angles.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_PATHANGLE_CRITIC_HPP__
#define __MAPPI_PATHANGLE_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class PathAngle : public Critic {

    // VARIABLES

    protected:
        mappi::config::PathAngleCritic cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new PathAngle critic
         * 
         */
        PathAngle();

         /**
         * @brief Configure the Critic
         * 
         * @param name Name of the critic
         * @param config Configuration object
         * @param costmap_ros Costmap 2D object
         */
        void configure(std::string name, 
                        mappi::config::
                            PathAngleCritic& config,
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
        void setConfig(mappi::config::PathAngleCritic&);
};

PathAngle::PathAngle() { }

void PathAngle::configure(std::string name, 
                        mappi::config::
                            PathAngleCritic& config,
                        mappi::shared_ptr
                            <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void PathAngle::score(mappi::objects::State& states,
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

    const size_t path_size = plan.x.shape(0) - 1;

    size_t min_dist_path_point = mappi::aux::findPathMinDistPoint(trajectories, plan);

    auto offseted_idx = std::min(min_dist_path_point + static_cast<size_t>(cfg_.offset_from_furthest), path_size);

    const auto path_x = plan.x(offseted_idx);
    const auto path_y = plan.y(offseted_idx);

    if ( std::fabs(mappi::aux::poseToPointAngle(states.odom, path_x, path_y)) < cfg_.angle_threshold ) 
        return;

    const auto yaws_between_points = xt::atan2(
        path_y - trajectories.y,
        path_x - trajectories.x);
    const auto yaws =
        xt::abs(mappi::aux::shortest_angular_dist(trajectories.yaw, yaws_between_points));

    using xt::evaluation_strategy::immediate;
    costs += xt::pow(xt::mean(yaws, {1}, immediate) * cfg_.common.weight, cfg_.common.power);

}

void PathAngle::setConfig(mappi::config::PathAngleCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics


#endif
