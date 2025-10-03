/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Goal Critic. It penalizes trajectories far from the goal.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_GOAL_CRITIC_HPP__
#define __MAPPI_GOAL_CRITIC_HPP__

#include "Critics/Critic.hpp"

namespace mappi::critics {

class Goal : public Critic {

    // VARIABLES

    protected:
        mappi::config::GenericCritic cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Goal critic
         * 
         */
        Goal();

        /**
         * @brief Configure the Critic
         * 
         * @param name Name of the critic
         * @param config Configuration object
         * @param costmap_ros Costmap 2D object
         */
        void configure(std::string name, 
                        mappi::config::
                            GenericCritic& config,
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
        void setConfig(mappi::config::GenericCritic&);
};

} // namespace mappi::critics


#endif
