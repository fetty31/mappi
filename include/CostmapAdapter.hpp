/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-09-29
 * 
 * Description :
 *   Nav2 Costmap adapter for mappi::critics::Critics class
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_NAV2_COSTMAP_ADAPTER_HPP__
#define __MAPPI_NAV2_COSTMAP_ADAPTER_HPP__

#include "mappi/utils/CostmapInterface.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.h>
#include <nav2_costmap_2d/costmap_2d.h>
#include <nav2_costmap_2d/inflation_layer.h>
#include <geometry_msgs/msg/point.hpp>

#include <string>
#include <cmath>
#include <numeric>

namespace mappi {

class ROS2CostmapAdapter : public mappi::utils::CostmapInterface {

    // typedef boost::recursive_mutex mutex_t;
    // typedef boost::unique_lock<mutex_t> unique_lock;

    // nav2_costmap_2d::Costmap2DROS* costmap_ros_;
    mappi::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

public:
    explicit ROS2CostmapAdapter(mappi::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap)
        : costmap_ros_(ros_costmap) {}

    unsigned char costAt(double x, double y) const override {
        unsigned int mx, my;
        auto* costmap = costmap_ros_->getCostmap();
        if (!costmap->worldToMap(x, y, mx, my)) // this point lies outside the costmap
            return nav2_costmap_2d::NO_INFORMATION; 
        return costmap->getCost(mx, my);
    }

    unsigned char costAt(double x, double y, double theta) const override {

        auto* costmap = costmap_ros_->getCostmap();
        // unique_lock lock(*(costmap->getMutex())); // To-Do: adapt to nav2 costmap

        unsigned int x_i, y_i;
        if(!costmap->worldToMap(x, y, x_i, y_i)) // this point lies outside the costmap
            return nav2_costmap_2d::NO_INFORMATION;

        unsigned char output_cost = nav2_costmap_2d::FREE_SPACE;

        std::vector<geometry_msgs::msg::Point> oriented_footprint = costmap_ros_->getOrientedFootprint();

        static unsigned int N = 5;
        for(unsigned int i=0; i < oriented_footprint.size(); i++){
            double p1_x = oriented_footprint[i].x;
            double p1_y = oriented_footprint[i].y;

            double p2_x = oriented_footprint[std::fmod(i+1,oriented_footprint.size())].x;
            double p2_y = oriented_footprint[std::fmod(i+1,oriented_footprint.size())].y;

            double slope_x = std::atan2( p2_y-p1_y, p2_x-p1_x ); // slope w.r.t x-axis
            double slope_y = std::atan2( p2_x-p1_x, p2_y-p1_y ); // slope w.r.t y-axis

            double delta_x = std::fabs(p2_x-p1_x)/static_cast<double>(N);
            double delta_y = std::fabs(p2_y-p1_y)/static_cast<double>(N);

            double sign_x = 1.0;
            double sign_y = 1.0;
            if(p2_x < p1_x)
                sign_x = -1.0;
            if(p2_y < p1_y)
                sign_y = -1.0;

            unsigned int mx, my;
            if(slope_x < slope_y){
                double x = p1_x;
                double y;
                for(unsigned int j=0; j < N; j++){
                    x += sign_x*delta_x;
                    y = p1_y + slope_x*(x-p1_x);

                    if(not costmap_->worldToMap(x, y, mx, my))
                        return nav2_costmap_2d::LETHAL_OBSTACLE;

                    output_cost = std::max(costmap_->getCost(mx, my), output_cost);
                }
            }
            else{
                double x;
                double y = p1_y;
                for(unsigned int j=0; j < N; j++){
                    y += sign_y*delta_y;
                    x = p1_x + slope_y*(y-p1_y);

                    if(not costmap_->worldToMap(x, y, mx, my))
                        return nav2_costmap_2d::LETHAL_OBSTACLE;

                    output_cost = std::max(costmap_->getCost(mx, my), output_cost);
                }
            }

        }

        return output_cost;
    }


    std::vector<mappi::utils::Point2D> getFootprint() const override {
        std::vector<mappi::utils::Point2D> fp;
        for (auto& pt : costmap_ros_->getRobotFootprint()) {
            fp.push_back({pt.x, pt.y});
        }
        return fp;
    }

    double getInscribedRadius() const override {
        return costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    }

    bool isInCollision(unsigned char cost) const override {
        return (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
    // bool isInCollision(unsigned char cost){
    //     bool is_tracking_unkown = 
    //         costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

    //     switch(cost) {
    //         case(nav2_costmap_2d::LETHAL_OBSTACLE):
    //             return true;
    //         case(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
    //             return true;
    //         case(nav2_costmap_2d::NO_INFORMATION):
    //             return is_tracking_unkown ? false : true;
    //         case(nav2_costmap_2d::FREE_SPACE):
    //             return false;
    //         default:
    //             return false;
    //     }
    // }
};

} // namespace mappi

#endif
