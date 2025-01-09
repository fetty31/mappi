#ifndef __NANO_MPPI_OBSTACLE_CRITIC_HPP__
#define __NANO_MPPI_OBSTACLE_CRITIC_HPP__

#include "Critics/Critic.hpp"

#include <boost/shared_ptr.hpp>

namespace nano_mppic::critics {

class Obstacles : public Critic {

    // VARIABLES

    protected:
        float inflation_radius_;
        float inflation_scale_factor_;
        float collision_cost_, collision_margin_dist_;
        float power_;
        float critical_weight_;

    // FUNCTIONS

    public:
        Obstacles();
        
        void configure(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros);

        void score(nano_mppic::objects::State& states,
                            nano_mppic::objects::Trajectory& trajectories,
                            xt::xtensor<float,1>& costs,
                            bool &all_traj_collide) override;

    private:
        unsigned char costAtPose(float x, float y, float yaw);

        bool isInCollision(unsigned char cost);

        float dist2obstacle(unsigned char cost);

        // void findCircumscribedCost();
};

Obstacles::Obstacles() : inflation_radius_(0.0f),
                inflation_scale_factor_(0.0f),
                collision_cost_(100000.0f),
                collision_margin_dist_(0.1f),
                critical_weight_(10.0f),
                power_(1.0f) { }

void Obstacles::configure(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function

    // findCircumscribedCost();
    /* To-Do:
        - get InflationRadius and InflationScaleFactor params
    */

    collision_cost_ = 10000.0f;
    collision_margin_dist_ = 0.1f;
    critical_weight_ = 10.0f;
    power_ = 2.0f;
}

void Obstacles::score(nano_mppic::objects::State& states,
                    nano_mppic::objects::Trajectory& trajectories,
                    xt::xtensor<float,1>& costs,
                    bool &fail_flag) 
{
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
            unsigned char cost_c = costAtPose(trajectories.x(i,j), trajectories.y(i,j), trajectories.yaw(i,j));
            if(cost_c < 1) // free space
                continue; 
            
            if(isInCollision(cost_c)){
                collision = true;
                break;
            }

            if(this->inflation_radius_ == 0.0f ||
                this->inflation_scale_factor_ == 0.0f ) continue; // cannot process repulsion cost (inflation layer doesn't exist)

            const float dist2obs = dist2obstacle(cost_c);

            if( dist2obs < collision_margin_dist_ )
                cost += (collision_margin_dist_ - dist2obs);
        }

        if(not collision) all_collide = false;
        raw_cost[i] = collision ? collision_cost_ : cost;
    }

    costs += xt::pow( (critical_weight_ * raw_cost), power_);
    fail_flag = all_collide;
}

unsigned char Obstacles::costAtPose(float x, float y, float yaw){
    unsigned int x_i, y_i;
    costmap_ptr_->worldToMap(x, y, x_i, y_i);
    
    return costmap_ptr_->getCost(x_i, y_i);
}

bool Obstacles::isInCollision(unsigned char cost){
    bool is_tracking_unkown = 
        costmap_ros_ptr_->getLayeredCostmap()->isTrackingUnknown();

    switch(cost) {
        case(costmap_2d::LETHAL_OBSTACLE):
            return true;
        case(costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
            return true;
        case(costmap_2d::NO_INFORMATION):
            return is_tracking_unkown ? false : true;
        case(costmap_2d::FREE_SPACE):
            return false;
        default:
            return false;
    }
}

float Obstacles::dist2obstacle(unsigned char cost){
    const float scale_factor = inflation_scale_factor_;
    const float min_radius = costmap_ros_ptr_->getLayeredCostmap()->getInscribedRadius();
    float dist_to_obs = (scale_factor * min_radius - log(static_cast<float>(cost)) + log(253.0f)) / scale_factor;

    return dist_to_obs;
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
//             inflation_scale_factor_ = static_cast<float>(inflation_layer->getCostScalingFactor());
//             inflation_radius_ = static_cast<float>(inflation_layer->getInflationRadius());
//     }

//     if(not inflation_layer_found)
//         std::cout << "NANO_MPPIC::OBSTACLES Error: No Inflation Layer found in Costmap2D!!\n";
// } 

} // namespace nano_mppic::critics

#endif