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

#include "Utils/Splines/BaseSpline.hpp"

namespace mappi::spline {

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
const std::size_t& BaseSpline<T>::size()
{
    return control_points_.size();
}

template<typename T>
const std::size_t& BaseSpline<T>::degree()
{
    return degree_;
}

} // namespace mappi::spline