/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Cubic B-spline class.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_CUBIC_BSPLINE_HPP__
#define __MAPPI_CUBIC_BSPLINE_HPP__

#include "Utils/Splines/BaseSpline.hpp" 

namespace mappi::spline {

template<typename T>
class CubicBSpline : public BaseSpline<T> {

    private:
        std::vector<T> knotVector_;

        /**
         * @brief Initializing function
         * 
         */
        void initialize() override;

        /**
         * @brief Find knot span
         * 
         * @param n 
         * @param u 
         * @return std::size_t 
         */
        std::size_t findKnotSpan(const std::size_t n, const T u);

        /**
         * @brief B-spline basis function
         * 
         * @param i 
         * @param p 
         * @param u 
         * @return const T 
         */
        const T basisFunction(const std::size_t i, const std::size_t p, const T u);
        
        /**
         * @brief B-spline derivative basis function
         * 
         * @param i 
         * @param p 
         * @param u 
         * @param d_order 
         * @return const T 
         */
        const T basisFunctionDerivative(const std::size_t i, const std::size_t p, const T u, const std::size_t d_order);

    public:
        /**
         * @brief Construct a new Cubic B Spline object
         * 
         */
        CubicBSpline();

        /**
         * @brief Construct a new Cubic B Spline object
         * 
         * @param control_points 
         */
        CubicBSpline(const std::vector<std::vector<T>>& control_points);

         /**
         * @brief Evaluate spline 
         * 
         * @param u Point where to eval
         * @param d_order Derivative order
         * @return const std::vector<T> 
         */
        const std::vector<T> evaluate(const T u, const std::size_t d_order) override;

        /**
         * @brief Evaluate spline at multiple points
         * 
         * @param u_vec Points where to eval
         * @param d_order Derivative order
         * @return const std::vector<std::vector<T>> 
         */
        const std::vector<std::vector<T>> evaluate(const std::vector<T> u_vec, const std::size_t d_order) override;
};

}// namespace mappi::spline

typedef mappi::spline::CubicBSpline<float> fCubicBSpline;
typedef mappi::spline::CubicBSpline<double> dCubicBSpline;

#endif