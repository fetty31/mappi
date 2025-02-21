#ifndef __MAPPI_OBSTACLE_CRITIC_HPP__
#define __MAPPI_OBSTACLE_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class Obstacles : public Critic {

    // VARIABLES

    protected:
        mappi::config::ObstaclesCritic cfg_;

    // FUNCTIONS

    public:
        Obstacles();
        
        void configure(std::string name, 
                        mappi::config::
                            ObstaclesCritic& config,
                        mappi::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &all_traj_collide) override;

        void setConfig(mappi::config::ObstaclesCritic&);

    private:

        float dist2obstacle(unsigned char cost);

};

Obstacles::Obstacles() { }

void Obstacles::configure(std::string name, 
                        mappi::config::
                            ObstaclesCritic& config,
                        mappi::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Obstacles::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) 
{
    if(not costmap_ros_ptr_ || not cfg_.common.active){
        return;
    }

    bool near_goal = false;
    if(mappi::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        near_goal = true;

    auto && raw_cost = xt::xtensor<float,1>::from_shape({costs.shape(0)});
    raw_cost.fill(0.0);
    auto && repulsive_cost = xt::xtensor<float,1>::from_shape({costs.shape(0)});
    repulsive_cost.fill(0.0);

    const size_t n_traj = trajectories.x.shape(1);
    bool all_collide = true;
    for(size_t i=0; i < trajectories.x.shape(0)/*batch_size*/; ++i){
        bool collision = false;
        float cost = 0.0;
        
        for(size_t j=0; j < n_traj; j++){
            unsigned char cost_c = this->costAtPose(trajectories.x(i,j), trajectories.y(i,j));

            if(cost_c < 1) // free space
                continue; 
            
            if(this->isInCollision(cost_c)){
                collision = true;
                break;
            }

            if(cfg_.inflation_radius == 0.0f ||
                cfg_.inflation_scale_factor == 0.0f ) continue; // cannot process repulsion cost (inflation layer doesn't exist)

            const float dist2obs = dist2obstacle(cost_c);

            if( dist2obs < cfg_.collision_margin_dist )
                cost += std::fabs(cfg_.collision_margin_dist - dist2obs);
            else if(not near_goal)
                repulsive_cost[i] += std::fabs(cfg_.inflation_radius - dist2obs);
        }

        if(not collision) all_collide = false;
        raw_cost[i] = collision ? cfg_.collision_cost : cost;
    }

    costs += xt::pow( 
        (cfg_.common.weight * raw_cost) + 
        (cfg_.repulsive_weight * repulsive_cost / n_traj), cfg_.common.power);
    fail_flag = all_collide;
}

float Obstacles::dist2obstacle(unsigned char cost){
    const float scale_factor = cfg_.inflation_scale_factor;
    const float min_radius = costmap_ros_ptr_->getLayeredCostmap()->getInscribedRadius();
    float dist_to_obs = (scale_factor * min_radius - log(static_cast<float>(cost)) + log(253.0f)) / scale_factor;

    return dist_to_obs;
}

void Obstacles::setConfig(mappi::config::ObstaclesCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics

#endif