#ifndef __NANO_MPPI_OBSTACLE_CRITIC_HPP__
#define __NANO_MPPI_OBSTACLE_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace nano_mppic::critics {

class Obstacles : public Critic {

    // VARIABLES

    protected:
        nano_mppic::config::ObstaclesCritic cfg_;

    // FUNCTIONS

    public:
        Obstacles();
        
        void configure(std::string name, 
                        nano_mppic::config::
                            ObstaclesCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &all_traj_collide) override;

        void setConfig(nano_mppic::config::ObstaclesCritic&);

    private:

        float dist2obstacle(unsigned char cost);

        // void findCircumscribedCost();
};

Obstacles::Obstacles() { }

void Obstacles::configure(std::string name, 
                        nano_mppic::config::
                            ObstaclesCritic& config,
                        nano_mppic::shared_ptr
                            <costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;

    // findCircumscribedCost();
    /* To-Do:
        - get InflationRadius and InflationScaleFactor params from costmapROS ???
    */
}

void Obstacles::score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    nano_mppic::objects::Path& plan,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) 
{
    std::cout << "NANO_MPPIC::MPPIc::Obstacles::score()\n";

    if(not costmap_ros_ptr_){
        std::cout << "NANO_MPPIC::OBSTACLES Error: no costmap object passed to critic function!\n";
        return;
    }

    auto && raw_cost = xt::xtensor<float,1>::from_shape({costs.shape(0)});
    raw_cost.fill(0.0);

    const size_t n_traj = trajectories.x.shape(1);
    bool all_collide = true;
    for(size_t i=0; i < trajectories.x.shape(0)/*batch_size*/; ++i){
        bool collision = false;
        float cost = 0.0;
        
        for(size_t j=0; j < n_traj; j++){
            unsigned char cost_c = this->costAtPose(trajectories.x(i,j), trajectories.y(i,j), trajectories.yaw(i,j));
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
                cost += (cfg_.collision_margin_dist - dist2obs);
        }

        if(collision){
            std::cout << "NANO_MPPIC::MPPIc::Obstacles collision detected!\n";
        }

        if(not collision) all_collide = false;
        raw_cost[i] = collision ? cfg_.collision_cost : cost;
    }

    costs += xt::pow( (cfg_.common.weight * raw_cost), cfg_.common.power);
    fail_flag = all_collide;
}

float Obstacles::dist2obstacle(unsigned char cost){
    const float scale_factor = cfg_.inflation_scale_factor;
    const float min_radius = costmap_ros_ptr_->getLayeredCostmap()->getInscribedRadius();
    float dist_to_obs = (scale_factor * min_radius - log(static_cast<float>(cost)) + log(253.0f)) / scale_factor;

    return dist_to_obs;
}

void Obstacles::setConfig(nano_mppic::config::ObstaclesCritic& config)
{
    cfg_ = config;
}

// void Obstacles::findCircumscribedCost(){
//     bool inflation_layer_found = false;
//     for(auto layer = costmap_ros_ptr_->getLayeredCostmap()->getPlugins()->begin();
//         layer != costmap_ros_ptr_->getLayeredCostmap()->getPlugins()->end();
//         layer++) {
//             auto inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
//             if(not inflation_layer)
//                 continue;

//             inflation_layer_found = true;
//             cfg_.inflation_scale_factor = static_cast<float>(inflation_layer->getCostScalingFactor());
//             cfg_.inflation_radius = static_cast<float>(inflation_layer->getInflationRadius());
//     }

//     if(not inflation_layer_found)
//         std::cout << "NANO_MPPIC::OBSTACLES Error: No Inflation Layer found in Costmap2D!!\n";
// } 

} // namespace nano_mppic::critics

#endif