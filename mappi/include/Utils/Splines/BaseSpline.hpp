/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Base Spline class.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_BASE_SPLINE_HPP__
#define __MAPPI_BASE_SPLINE_HPP__

#include <vector>
#include <cstddef>
#include <cmath>

namespace mappi::spline {

template<typename T>
class BaseSpline {

    protected:
        std::size_t degree_;
        std::vector<std::vector<T>> control_points_;

        /**
         * @brief Initializing function
         * 
         */
        virtual void initialize() = 0;

    public:
        /**
        * @brief Construct a new Base Spline object
        * 
        */
        BaseSpline();

        /**
         * @brief Destroy the Base Spline object
         * 
         */
        ~BaseSpline() = default;

        /**
         * @brief Construct a new Base Spline object
         * 
         * @param control_points Control points of the B-spline
         */
        BaseSpline(const std::vector<std::vector<T>>& control_points);

        /**
         * @brief Evaluate spline 
         * 
         * @param u Point where to eval
         * @param d_order Derivative order
         * @return const std::vector<T> 
         */
        virtual const std::vector<T> evaluate(const T u, const std::size_t d_order) = 0;

        /**
         * @brief Evaluate spline at multiple points
         * 
         * @param u_vec Points where to eval
         * @param d_order Derivative order
         * @return const std::vector<std::vector<T>> 
         */
        virtual const std::vector<std::vector<T>> evaluate(const std::vector<T> u_vec, const std::size_t d_order) = 0;

        /**
         * @brief Compute curvature of spline
         * 
         * @param u Point where to eval
         * @return const T 
         */
        virtual const T curvature(const T u);

        /**
         * @brief Set the control points
         * 
         * @param control_points 
         */
        void setControlPoints(const std::vector<std::vector<T>>& control_points);

        /**
         * @brief Get the control points
         * 
         * @return const std::vector<std::vector<T>>& 
         */
        const std::vector<std::vector<T>>& getControlPoints();

        /**
         * @brief Get spline size
         * 
         * @return const std::size_t 
         */
        std::size_t size() const;

        /**
         * @brief Get spline degree
         * 
         * @return const std::size_t& 
         */
        std::size_t degree() const;
};

}// namespace mappi::spline

typedef mappi::spline::BaseSpline<float> fBaseSpline;
typedef mappi::spline::BaseSpline<double> dBaseSpline;

#endif