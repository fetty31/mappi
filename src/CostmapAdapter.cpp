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

// Collision-aware cost using circle
unsigned char ROS2CostmapAdapter::costAtCircle(double x, double y, double radius)
{
    auto* costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

    // Convert the circle radius into map cells
    unsigned int mx_center, my_center;
    if (!costmap->worldToMap(x, y, mx_center, my_center))
        return nav2_costmap_2d::NO_INFORMATION;

    const double resolution = costmap->getResolution();
    int cell_radius = static_cast<int>(std::ceil(radius / resolution));

    unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;

    // Iterate over all cells in the bounding square of the circle
    for (int dx = -cell_radius; dx <= cell_radius; ++dx)
    {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy)
        {
            int mx = static_cast<int>(mx_center) + dx;
            int my = static_cast<int>(my_center) + dy;

            // Skip cells outside the circle
            double dist = std::hypot(dx * resolution, dy * resolution);
            if (dist > radius)
                continue;

            // Skip cells outside the map
            if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap->getSizeInCellsX()) || 
                my >= static_cast<int>(costmap->getSizeInCellsY()))
                return nav2_costmap_2d::NO_INFORMATION;

            unsigned char cost = costmap->getCost(mx, my);
            max_cost = std::max(max_cost, cost);

            if (max_cost == nav2_costmap_2d::LETHAL_OBSTACLE)  // early exit
                return max_cost;
        }
    }

    return max_cost;
}


// Collision-aware cost using footprint
unsigned char ROS2CostmapAdapter::costAt(double x, double y, double theta) 
{
    auto* costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

    // Precompute sin/cos for efficiency
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    const auto& base_fp = costmap_ros_->getRobotFootprint();
    std::vector<geometry_msgs::msg::Point> oriented_footprint;
    oriented_footprint.reserve(base_fp.size());

    for (const auto& pt : base_fp) {
        geometry_msgs::msg::Point p;
        p.x = x + pt.x * cos_theta - pt.y * sin_theta;
        p.y = y + pt.x * sin_theta + pt.y * cos_theta;
        p.z = 0.0;
        oriented_footprint.push_back(p);
    }

    unsigned char max_cost = nav2_costmap_2d::FREE_SPACE;
    const unsigned int N = 10;

    for (size_t i = 0; i < oriented_footprint.size(); ++i) {
        const auto& p1 = oriented_footprint[i];
        const auto& p2 = oriented_footprint[(i + 1) % oriented_footprint.size()];

        const double dx = (p2.x - p1.x) / N;
        const double dy = (p2.y - p1.y) / N;

        for (unsigned int j = 0; j <= N; ++j) {
            double px = p1.x + j * dx;
            double py = p1.y + j * dy;

            unsigned int mx, my;
            if (!costmap->worldToMap(px, py, mx, my))
                return nav2_costmap_2d::NO_INFORMATION;

            max_cost = std::max(max_cost, costmap->getCost(mx, my));
            if (max_cost == nav2_costmap_2d::LETHAL_OBSTACLE)  // early exit
                return max_cost;
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