#ifndef __NANO_MPPI_SPLINE_ADAPTOR_HPP__
#define __NANO_MPPI_SPLINE_ADAPTOR_HPP__

#include "Objects/Trajectory.hpp"
#include "Utils/Splines/CubicBSpline.hpp"

namespace nano_mppic::spline {

class BSpline : public fCubicBSpline {

    public:
        BSpline(const nano_mppic::objects::Path& path) : fCubicBSpline()
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

        nano_mppic::objects::Path interpolate(const std::vector<float> u_vec, const size_t d_order) 
        {
            const std::vector<std::vector<float>> interp_points = fCubicBSpline::evaluate(u_vec, d_order);

            nano_mppic::objects::Path path;
            path.reset(interp_points.size());
            for(size_t i=0; i < interp_points.size(); i++){
                path.x(i) = interp_points[i][0];
                path.y(i) = interp_points[i][1];
                path.yaw(i) = interp_points[i][2];
            }

            return path;
        }

        const std::vector<float> interpolate(const float u, const size_t d_order)
        {
            return fCubicBSpline::evaluate(u, d_order);
        }

};

} // namespace nano_mppic::spline

#endif