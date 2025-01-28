#ifndef __NANO_MPPI_PATHANGLE_CRITIC_HPP__
#define __NANO_MPPI_PATHANGLE_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class PathAngle : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::PathCritic cfg_;

    // FUNCTIONS

    public:
        PathAngle();
        
        void configure(std::string name, 
                        nano_mppic::config::
                            PathCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) override;

        void setConfig(nano_mppic::config::PathCritic&);
};

PathAngle::PathAngle() { }

void PathAngle::configure(std::string name, 
                        nano_mppic::config::
                            PathCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void PathAngle::score(nano_mppic::objects::State& states,
                        nano_mppic::objects::Trajectory& trajectories,
                        nano_mppic::objects::Path& plan,
                        xt::xtensor<float,1>& costs,
                        bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
        std::cout << "NANO_MPPIC::PathAngle critic not active\n";
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

    const auto yaws_between_points = xt::atan2(
        path_y - trajectories.y,
        path_x - trajectories.x);
    const auto yaws =
        xt::abs(nano_mppic::aux::angularDist(trajectories.yaw, yaws_between_points));

    using xt::evaluation_strategy::immediate;
    costs += xt::pow(xt::mean(yaws, {1}, immediate) * cfg_.common.weight, cfg_.common.power);

}

void PathAngle::setConfig(nano_mppic::config::PathCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
