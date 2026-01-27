/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Costmap abstraction.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_COSTMAP_INFERFACE_HPP__
#define __MAPPI_COSTMAP_INFERFACE_HPP__

#include <vector>

namespace mappi::utils {

// Minimal footprint-independent 2D point
struct Point2D {
    double x;
    double y;
};

class CostmapInterface {
public:
    virtual ~CostmapInterface() = default;

    virtual unsigned char costAt(double x, double y) const = 0;
    virtual unsigned char costAt(double x, double y, double theta) = 0;
    virtual unsigned char costAtCircle(double x, double y, double radius) = 0;

    virtual std::vector<Point2D> getFootprint() const = 0;
    virtual bool isInCollision(unsigned char cost) const = 0;
    virtual bool isInLethalCollision(unsigned char cost) const = 0;
    virtual double getInscribedRadius() const = 0;
};

} // namespace mappi::core

#endif