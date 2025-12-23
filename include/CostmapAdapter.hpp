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

#include "mppic.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace mappi {

class ROS2CostmapAdapter : public mappi::utils::CostmapInterface {

    mappi::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

public:
    explicit ROS2CostmapAdapter(mappi::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap);

    unsigned char costAt(double x, double y) const override;
    unsigned char costAt(double x, double y, double theta) override;

    std::vector<mappi::utils::Point2D> getFootprint() const override;
    double getInscribedRadius() const override;
    bool isInCollision(unsigned char cost) const override;
    bool isInLethalCollision(unsigned char cost) const override;
};

} // namespace mappi

#endif
