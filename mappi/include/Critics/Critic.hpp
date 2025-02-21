#ifndef __MAPPI_CRITIC_HPP__
#define __MAPPI_CRITIC_HPP__

#include "Objects/Config.hpp"
#include "Utils/SharedPtr.hpp"

#include <string>
#include <cmath>

#include <costmap_2d/costmap_2d_ros.h> // to-do: avoid ros dependency
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>

namespace mappi::critics {

class Critic {

    // VARIABLES

    protected:
        std::string name_;
        mappi::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_ptr_;
        costmap_2d::Costmap2D * costmap_ptr_{nullptr};

    // FUNCTIONS

    public:
        Critic() = default;

        virtual ~Critic() {delete costmap_ptr_;};

        virtual void configure(std::string name, mappi::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros){
            name_ = name;
            costmap_ros_ptr_ = costmap_ros;
            costmap_ptr_ = costmap_ros->getCostmap();
        }

        virtual void score(mappi::objects::State&,
                            mappi::objects::Trajectory&,
                            mappi::objects::Path&,
                            xt::xtensor<float,1>&,
                            bool& ) = 0;

        std::string getName(){ return name_; }
    
        unsigned char costAtPose(float x, float y){
            unsigned int x_i, y_i;
            if(not costmap_ptr_->worldToMap(x, y, x_i, y_i)) // this point lies outside the costmap
                return 0;

            return costmap_ptr_->getCost(x_i, y_i);
        }

        bool isInCollision(unsigned char cost){
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
};

} // namespace mappi::critics

#endif