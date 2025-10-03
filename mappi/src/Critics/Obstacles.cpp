/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Obstacles Critic. It penalizes trajectories using the costmap information.
 *
 * -----------------------------------------------------------------------------
 */

#include "Critics/Obstacles.hpp"

namespace mappi::critics {

Obstacles::Obstacles() { }

void Obstacles::configure(std::string name, 
                        mappi::config::
                            ObstaclesCritic& config,
                        mappi::shared_ptr
                            <mappi::utils::CostmapInterface>& costmap_ros){

    Critic::configure(name, costmap_ros); // call parent function
    cfg_ = config;
}

void Obstacles::score(mappi::objects::State& states,
                    mappi::objects::Trajectory& trajectories,
                    mappi::objects::Path& plan,
                    xt::xtensor<double,1>& costs,
                    bool &fail_flag) 
{
    if(not costmap_ || not cfg_.common.active){
        return;
    }

    bool near_goal = false;
    if(mappi::aux::robotNearGoal(cfg_.common.threshold, states.odom, plan))
        near_goal = true;

    auto && raw_cost = xt::xtensor<double,1>::from_shape({costs.shape(0)});
    raw_cost.fill(0.0);
    auto && repulsive_cost = xt::xtensor<double,1>::from_shape({costs.shape(0)});
    repulsive_cost.fill(0.0);

    const size_t n_traj = trajectories.x.shape(1);
    bool all_collide = true;
    for(size_t i=0; i < trajectories.x.shape(0)/*batch_size*/; ++i){
        bool collision = false;
        double cost = 0.0;
        
        for(size_t j=0; j < n_traj; j++){
            unsigned char cost_c = this->costAt(trajectories.x(i,j), trajectories.y(i,j));
            // unsigned char cost_c = this->costAt(trajectories.x(i,j), trajectories.y(i,j), trajectories.yaw(i,j));

            if(cost_c < 1) // free space
                continue; 
            
            if(this->isInCollision(cost_c)){
                collision = true;
                break;
            }

            if(cfg_.inflation_radius == 0.0 ||
                cfg_.inflation_scale_factor == 0.0 ) continue; // cannot process repulsion cost (inflation layer doesn't exist)

            const double dist2obs = dist2obstacle(cost_c);

            if( dist2obs < cfg_.collision_margin_dist )
                cost += cfg_.collision_margin_dist - dist2obs;
            else if(not near_goal)
                repulsive_cost[i] += cfg_.inflation_radius - dist2obs;
        }

        if(not collision) all_collide = false;
        raw_cost[i] = collision ? cfg_.collision_cost : cost;
    }

    costs += xt::pow( 
        cfg_.common.weight * raw_cost, cfg_.common.power);
    costs += xt::pow( 
        cfg_.repulsive_weight * repulsive_cost / n_traj, cfg_.common.power);
        
    fail_flag = all_collide;
}

double Obstacles::dist2obstacle(unsigned char cost){
    const double scale_factor = cfg_.inflation_scale_factor;
    const double min_radius = costmap_->getInscribedRadius();

    // NOTE: cost = exp(-scale_factor * (dist2obs - radius))*(INSCRIBED_INFLATED_OBSTACLE-1)
    //   (where INSCRIBED_INFLATED_OBSTACLE == 254) 

    // so in order to retrieve the distance:
    double dist_to_obs = (log(253.0) - log(static_cast<double>(cost))) / scale_factor + min_radius;
    dist_to_obs -= min_radius; // remove inscribed radius (because we are not doing any footprint check)

    return dist_to_obs;
}

void Obstacles::setConfig(mappi::config::ObstaclesCritic& config)
{
    cfg_ = config;
}

} // namespace mappi::critics