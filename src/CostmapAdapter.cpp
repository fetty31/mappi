#include "CostmapAdapter.hpp"
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <mutex>

namespace mappi {

// Constructor
ROS2CostmapAdapter::ROS2CostmapAdapter(
    mappi::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap)
: costmap_ros_(ros_costmap)
{}

// Simple cost lookup
unsigned char ROS2CostmapAdapter::costAt(double x, double y) const
{
    auto* costmap = costmap_ros_->getCostmap();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

    unsigned int mx, my;
    if (!costmap->worldToMap(x, y, mx, my))
        return nav2_costmap_2d::NO_INFORMATION;

    return costmap->getCost(mx, my);
}

// Collision-aware cost using footprint
unsigned char ROS2CostmapAdapter::costAt(double x, double y, double theta) 
{
    auto* costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

    // Transform footprint according to x, y, theta
    std::vector<geometry_msgs::msg::Point> oriented_footprint;
    auto base_fp = costmap_ros_->getRobotFootprint();
    for (const auto& pt : base_fp) {
        geometry_msgs::msg::Point p;
        p.x = x + (pt.x * cos(theta) - pt.y * sin(theta));
        p.y = y + (pt.x * sin(theta) + pt.y * cos(theta));
        p.z = 0.0;
        oriented_footprint.push_back(p);
    }

    unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
    const unsigned int N = 5;

    for (size_t i = 0; i < oriented_footprint.size(); ++i) {
        const auto& p1 = oriented_footprint[i];
        const auto& p2 = oriented_footprint[(i + 1) % oriented_footprint.size()];

        for (unsigned int j = 0; j <= N; ++j) {
            double t = j / static_cast<double>(N);
            double px = p1.x + t * (p2.x - p1.x);
            double py = p1.y + t * (p2.y - p1.y);

            unsigned int mx, my;
            if (!costmap->worldToMap(px, py, mx, my))
                return nav2_costmap_2d::NO_INFORMATION;

            max_cost = std::max(max_cost, costmap->getCost(mx, my));
        }
    }

    return max_cost;
}

// Footprint as 2D points
std::vector<mappi::utils::Point2D> ROS2CostmapAdapter::getFootprint() const
{
    std::vector<mappi::utils::Point2D> fp;
    for (auto& pt : costmap_ros_->getRobotFootprint()) {
        fp.push_back({pt.x, pt.y});
    }
    return fp;
}

// Inscribed radius
double ROS2CostmapAdapter::getInscribedRadius() const
{
    return costmap_ros_->getLayeredCostmap()->getInscribedRadius();
}

// Check if cost indicates collision
bool ROS2CostmapAdapter::isInCollision(unsigned char cost) const
{
    return (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

// Check if cost indicates lethal collision
bool ROS2CostmapAdapter::isInLethalCollision(unsigned char cost) const
{
    return (cost > nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

} // namespace mappi