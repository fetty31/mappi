#ifndef __NANO_MPPI_PATHFOLLOW_CRITIC_HPP__
#define __NANO_MPPI_PATHFOLLOW_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class PathFollow : public Critic {

    // VARIABLES

    protected:
        unsigned int power_;
        float weight_;

        float threshold_; 
        size_t offset_from_furthest_;

    // FUNCTIONS

    public:
        PathFollow();
        
        void configure(std::string name, 
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &all_traj_collide) override;

};

PathFollow::PathFollow() : power_(1),
                        weight_(5.0f),
                        threshold_(1.0f),
                        offset_from_furthest_(20) { }

void PathFollow::configure(std::string name, 
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function

    /* To-Do:
        - update parameters
    */
}

void PathFollow::score(nano_mppic::objects::State& states,
                        nano_mppic::objects::Trajectory& trajectories,
                        nano_mppic::objects::Path& plan,
                        xt::xtensor<float,1>& costs,
                        bool &all_traj_collide)
{

    std::cout << "NANO_MPPIC::MPPIc::PathFollow::score()\n";

    if(not costmap_ros_ptr_){
        std::cout << "NANO_MPPIC::PathFollow Error: no costmap object passed to critic function!\n";
        return;
    }

    /*To-Do:
        - check if we are closer than this->threshold_ to the trajectory
            . if closer -> do not compute any cost (return)
            . else -> pass
    */

    const size_t path_size = plan.x.shape(0) - 1;

    size_t min_dist_path_point = nano_mppic::aux::findPathMinDistPoint(trajectories, plan);

    auto offseted_idx = std::min(min_dist_path_point + offset_from_furthest_, path_size);

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

    costs += xt::pow(weight_ * std::move(dists), power_);
    
}

} // namespace nano_mppic::critics


#endif
