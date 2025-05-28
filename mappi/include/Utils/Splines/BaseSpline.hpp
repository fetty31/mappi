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

namespace mappi::spline {

template<typename T>
class BaseSpline {

    protected:
        size_t degree_;
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
        virtual const std::vector<T> evaluate(const T u, const size_t d_order) = 0;

        /**
         * @brief Evaluate spline at multiple points
         * 
         * @param u_vec Points where to eval
         * @param d_order Derivative order
         * @return const std::vector<std::vector<T>> 
         */
        virtual const std::vector<std::vector<T>> evaluate(const std::vector<T> u_vec, const size_t d_order) = 0;

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
         * @return const size_t 
         */
        const size_t size();

        /**
         * @brief Get spline degree
         * 
         * @return const size_t& 
         */
        const size_t& degree();
};

template<typename T>
BaseSpline<T>::BaseSpline() : degree_(3) { }

template<typename T>
BaseSpline<T>::BaseSpline(const std::vector<std::vector<T>>& control_points)
                                                : control_points_(control_points), degree_(3) 
{ 
    initialize();
}

template<typename T>
const std::vector<std::vector<T>>& BaseSpline<T>::getControlPoints()
{
    return control_points_;
}

template<typename T>
void BaseSpline<T>::setControlPoints(const std::vector<std::vector<T>>& control_points)
{
    control_points_ = control_points;
    initialize();
}

template<typename T>
const T BaseSpline<T>::curvature(const T u) 
{
    const auto firstDerivative = evaluate(u, 1);
    const auto secondDerivative = evaluate(u, 2);
    T numerator = firstDerivative[0] * secondDerivative[1] - firstDerivative[1] * secondDerivative[0];
    T denominator = std::pow(firstDerivative[0] * firstDerivative[0] + firstDerivative[1] * firstDerivative[1], 1.5);
    
    return std::fabs(numerator) / denominator;
}

template<typename T>
const size_t BaseSpline<T>::size()
{
    return control_points_.size();
}

template<typename T>
const size_t& BaseSpline<T>::degree()
{
    return degree_;
}

}// namespace mappi::spline

#endif