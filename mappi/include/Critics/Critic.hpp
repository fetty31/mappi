#ifndef __MAPPI_CRITIC_HPP__
#define __MAPPI_CRITIC_HPP__

#include "Objects/Config.hpp"
#include "Utils/SharedPtr.hpp"

#include <string>
#include <cmath>
#include <numeric>

#include <costmap_2d/costmap_2d_ros.h> // to-do: avoid ros dependency
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>

namespace mappi::critics {

class Critic {

    typedef boost::recursive_mutex mutex_t;
    typedef boost::unique_lock<mutex_t> unique_lock;

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
            
            unique_lock lock(*(costmap_ptr_->getMutex()));

            unsigned int x_i, y_i;
            if(not costmap_ptr_->worldToMap(x, y, x_i, y_i)) // this point lies outside the costmap
                return 0;

            return costmap_ptr_->getCost(x_i, y_i);
        }

        unsigned char costAtPose(float x, float y, float theta){

            unique_lock lock(*(costmap_ptr_->getMutex()));

            unsigned int x_i, y_i;
            if(not costmap_ptr_->worldToMap(x, y, x_i, y_i)) // this point lies outside the costmap
                return 0;

            unsigned char output_cost = costmap_2d::FREE_SPACE;

            // Check footprint collision
            float cos_th = cos(theta);
            float sin_th = sin(theta);

            std::vector<geometry_msgs::Point> footprint = costmap_ros_ptr_->getRobotFootprint();

            std::vector<geometry_msgs::Point> oriented_footprint;
            oriented_footprint.reserve(footprint.size());

            geometry_msgs::Point new_pt;
            for(unsigned int i = 0; i < footprint.size(); ++i) {
                new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
                new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
                oriented_footprint.push_back(new_pt);
            }

            static unsigned int N = 5;
            for(unsigned int i=0; i < oriented_footprint.size(); i++){
                float p1_x = oriented_footprint[i].x;
                float p1_y = oriented_footprint[i].y;

                float p2_x = oriented_footprint[std::fmod(i+1,oriented_footprint.size())].x;
                float p2_y = oriented_footprint[std::fmod(i+1,oriented_footprint.size())].y;

                float slope_x = std::atan2( p2_y-p1_y, p2_x-p1_x ); // slope w.r.t x-axis
                float slope_y = std::atan2( p2_x-p1_x, p2_y-p1_y ); // slope w.r.t y-axis

                float delta_x = std::fabs(p2_x-p1_x)/static_cast<float>(N);
                float delta_y = std::fabs(p2_y-p1_y)/static_cast<float>(N);

                float sign_x = 1.0;
                float sign_y = 1.0;
                if(p2_x < p1_x)
                    sign_x = -1.0;
                if(p2_y < p1_y)
                    sign_y = -1.0;

                unsigned int mx, my;
                if(slope_x < slope_y){
                    float x = p1_x;
                    float y;
                    for(unsigned int j=0; j < N; j++){
                        x += sign_x*delta_x;
                        y = p1_y + slope_x*(x-p1_x);

                        if(not costmap_ptr_->worldToMap(x, y, mx, my))
                            return costmap_2d::LETHAL_OBSTACLE;

                        output_cost = std::max(costmap_ptr_->getCost(mx, my), output_cost);
                    }
                }
                else{
                    float x;
                    float y = p1_y;
                    for(unsigned int j=0; j < N; j++){
                        y += sign_y*delta_y;
                        x = p1_x + slope_y*(y-p1_y);

                        if(not costmap_ptr_->worldToMap(x, y, mx, my))
                            return costmap_2d::LETHAL_OBSTACLE;

                        output_cost = std::max(costmap_ptr_->getCost(mx, my), output_cost);
                    }
                }

            }

            return output_cost;
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