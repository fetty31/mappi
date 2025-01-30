#ifndef __NANO_MPPI_SPLINE_ADAPTOR_HPP__
#define __NANO_MPPI_SPLINE_ADAPTOR_HPP__

#include "Objects/Trajectory.hpp"
#include "Utils/CubicBSpline.hpp"

namespace nano_mppic::spline {

class BSplineMPPI : public fCubicBSpline {

    public:
        BSplineMPPI(const nano_mppic::objects::Path& path) : fCubicBSpline()
        {
            std::vector<std::array<float,2>> points;
            points.reserve(path.x.size());

            std::array<float,2> arr;
            for(size_t i=0; i < path.x.size(); i++){
                arr[0] = path.x(i);
                arr[1] = path.y(i);
                points.push_back(arr);
            }

            this->setControlPoints(points);
            this->initialize();
        }

        const nano_mppic::objects::Path evaluate(const std::vector<float> u_vec, const size_t d_order) 
        {
            const std::vector<std::array<float,2>> interp_points = fCubicBSpline::evaluate(u_vec, d_order);

            nano_mppic::objects::Path path;
            path.reset(interp_points.size());
            for(size_t i=0; i < interp_points.size(); i++){
                path.x(i) = interp_points[i][0];
                path.y(i) = interp_points[i][1];
            }

            return path;
        }

        const std::array<float,2> evaluate(const float u, const size_t d_order)
        {
            return fCubicBSpline::evaluate(u, d_order);
        }

};

} // namespace nano_mppic::spline

#endif