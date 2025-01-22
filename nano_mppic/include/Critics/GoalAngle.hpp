#ifndef __NANO_MPPI_GOALANGLE_CRITIC_HPP__
#define __NANO_MPPI_GOALANGLE_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class GoalAngle : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        GoalAngle();
        
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

GoalAngle::GoalAngle() { }

void GoalAngle::configure(std::string name, 
                    nano_mppic::config::
                        GenericCritic& config,
                    nano_mppic::shared_ptr
                        <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void GoalAngle::score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag)
{

    if(not costmap_ros_ptr_ || not cfg_.common.active){
        std::cout << "NANO_MPPIC::GoalAngle critic not active\n";
        return;
    }

    if(not nano_mppic::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        return;

    const auto goal_idx = plan.x.shape(0) - 1;
    const float goal_yaw = plan.yaw(goal_idx);

    costs += xt::pow(
        xt::mean(xt::abs(nano_mppic::aux::angularDist(trajectories.yaw, goal_yaw)), {1}) * cfg_.common.weight, cfg_.common.power);
}

void GoalAngle::setConfig(nano_mppic::config::GenericCritic& config)
{
    cfg_ = config;
}

} // namespace nano_mppic::critics


#endif
