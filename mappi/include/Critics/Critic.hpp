/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Base class for the MPPI critics.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_CRITIC_HPP__
#define __MAPPI_CRITIC_HPP__

#include "Objects/Config.hpp"
#include "Utils/SharedPtr.hpp"
#include "Utils/CostmapInterface.hpp"

#include <string>
#include <cmath>
#include <numeric>

namespace mappi::critics {

class Critic {

    // VARIABLES

    protected:
        std::string name_;
        // const mappi::utils::CostmapInterface* costmap_; 
        mappi::shared_ptr<mappi::utils::CostmapInterface> costmap_;  

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Critic object
         * 
         */
        Critic() = default;

        /**
         * @brief Destroy the Critic object
         * 
         */
        virtual ~Critic() = default;

        /**
         * @brief Configure the Critic
         * 
         * @param name Name of the critic
         * @param costmap_ros Costmap 2D object
         */
        virtual void configure(std::string name, mappi::shared_ptr<mappi::utils::CostmapInterface>& costmap){
            name_ = std::move(name);
            costmap_ = costmap;
        }

        /**
         * @brief Score the sampled trajectories
         * 
         * @param states Current states
         * @param trajectories Sampled trajectories
         * @param plan Reference path
         * @param costs Output cost for each trajectory
         * @param fail_flag Output fail flag. True if all trajectories collide (or are unfeasible) 
         */
        virtual void score(mappi::objects::State&,
                            mappi::objects::Trajectory&,
                            mappi::objects::Path&,
                            xt::xtensor<double,1>&,
                            bool& ) = 0;
        
        /**
         * @brief Get the name of the critic
         * 
         * @return std::string 
         */
        std::string getName(){ return name_; }
        
        /**
         * @brief Get the costmap cost at specified point
         * 
         * @param x X coordinate [m]
         * @param y Y coordinate [m]
         * @return unsigned char 
         */
        unsigned char costAt(double x, double y) const {
            return costmap_->costAt(x, y);
        }

        /**
         * @brief Get the costmap cost at the specified 2D pose. Takes into account the robot's footprint.
         * 
         * @param x X coordinate [m]
         * @param y Y coordinate [m]
         * @param theta Yaw angle [rad]
         * @return unsigned char 
         */
        unsigned char costAt(double x, double y, double theta) const {
            return costmap_->costAt(x, y, theta);
        }

        /**
         * @brief Check if a cost value is in collision
         * 
         * @param cost Costmap cost [0-255]
         * @return true 
         * @return false 
         */
        bool isInCollision(unsigned char cost){
            return costmap_->isInCollision(cost);
        }

};

} // namespace mappi::critics

#endif