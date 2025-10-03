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
        /**
         * @brief Construct a new Obstacles critic
         * 
         */
        Obstacles();

         /**
         * @brief Configure the Critic
         * 
         * @param name Name of the critic
         * @param config Configuration object
         * @param costmap_ros Costmap 2D object
         */
        void configure(std::string name, 
                        mappi::config::
                            ObstaclesCritic& config,
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
                    bool &all_traj_collide) override;

        /**
         * @brief Overwrite the current configuration
         * 
         * @param config New configuration
         */
        void setConfig(mappi::config::ObstaclesCritic&);

    private:
        /**
         * @brief Get distance to obstacle from costmap cost
         * 
         * @param cost Costmap cost
         * @return double 
         */
        double dist2obstacle(unsigned char cost);

};

} // namespace mappi::critics

#endif