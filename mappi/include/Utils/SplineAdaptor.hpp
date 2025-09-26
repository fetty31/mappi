/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   B-spline class. Handles type conversion between MaPPI and Base Splines class.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_SPLINE_ADAPTOR_HPP__
#define __MAPPI_SPLINE_ADAPTOR_HPP__

#include "Objects/Trajectory.hpp"
#include "Utils/Splines/CubicBSpline.hpp"

namespace mappi::spline {

class BSpline : public fCubicBSpline {

    public:
        /**
         * @brief Construct a new BSpline object
         * 
         * @param path Path to interpolate
         */
        BSpline(const mappi::objects::Path& path) : fCubicBSpline()
        {
            std::vector<std::vector<float>> points;
            points.reserve(path.x.size());

            std::vector<float> p;
            p.resize(3);
            for(size_t i=0; i < path.x.size(); i++){
                p[0] = path.x(i);
                p[1] = path.y(i);
                p[2] = path.yaw(i);
                points.push_back(p);
            }

            this->setControlPoints(points);
        }

        /**
         * @brief Evaluate spline at multiple points
         * 
         * @param u_vec Points where to evaluate the spline
         * @param d_order Derivative order (0: spline / 1: first derivative / 2: second derivate / ...)
         * @return mappi::objects::Path 
         */
        mappi::objects::Path interpolate(const std::vector<float> u_vec, const size_t d_order) 
        {
            const std::vector<std::vector<float>> interp_points = fCubicBSpline::evaluate(u_vec, d_order);

            mappi::objects::Path path;
            path.reset(interp_points.size());
            for(size_t i=0; i < interp_points.size(); i++){
                path.x(i) = interp_points[i][0];
                path.y(i) = interp_points[i][1];
                path.yaw(i) = interp_points[i][2];
            }

            return path;
        }

        /**
         * @brief Evaluate spline
         * 
         * @param u Point where to evaluate
         * @param d_order Derivative order (0: spline / 1: first derivative / 2: second derivate / ...)
         * @return const std::vector<float> 
         */
        const std::vector<float> interpolate(const float u, const size_t d_order)
        {
            return fCubicBSpline::evaluate(u, d_order);
        }

};

} // namespace mappi::spline

#endif