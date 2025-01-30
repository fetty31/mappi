#ifndef __NANO_MPPI_CUBIC_BSPLINE_HPP__
#define __NANO_MPPI_CUBIC_BSPLINE_HPP__

#include <vector>

namespace nano_mppic::spline {

template<typename T>
class CubicBSpline {

    private:
        size_t degree_;
        std::vector<std::array<T,2>> control_points_;
        std::vector<T> knotVector_;

    public:
        CubicBSpline();
        CubicBSpline(const std::vector<std::array<T,2>>& control_points);

        const std::array<T,2> evaluate(const T u, const size_t d_order);
        const std::vector<std::array<T,2>> evaluate(const std::vector<T> u_vec, const size_t d_order);
        const T curvature(const T u);

        void initialize();
        const size_t findKnotSpan(const size_t n, const T u);
        const T basisFunction(const size_t i, const size_t p, const T u);
        const T basisFunctionDerivative(const size_t i, const size_t p, const T u, const size_t d_order);

        void setControlPoints(const std::vector<std::array<T,2>>& control_points);
        const std::vector<std::array<T,2>>& getControlPoints();

        const size_t size();
        const size_t& degree();
};

template<typename T>
CubicBSpline<T>::CubicBSpline() : degree_(3) { }

template<typename T>
CubicBSpline<T>::CubicBSpline(const std::vector<std::array<T,2>>& control_points)
                                                : control_points_(control_points), degree_(3) 
{ 
    initialize();
}

template<typename T>
const size_t CubicBSpline<T>::size()
{
    return control_points_.size();
}

template<typename T>
const size_t& CubicBSpline<T>::degree()
{
    return degree_;
}

template<typename T>
const std::vector<std::array<T,2>>& CubicBSpline<T>::getControlPoints()
{
    return control_points_;
}

template<typename T>
void CubicBSpline<T>::setControlPoints(const std::vector<std::array<T,2>>& control_points)
{
    control_points_ = control_points;

}

template<typename T>
void CubicBSpline<T>::initialize()
{
    const size_t numcontrol_points = control_points_.size();
    const size_t numKnots = numcontrol_points + degree_ + 1;
    knotVector_.reserve(numKnots);

    for (size_t i = 0; i <= degree_; ++i) {
        knotVector_[i] = 0.0;
    }

    for (size_t i = degree_ + 1; i < numKnots - degree_ - 1; ++i) {
        knotVector_[i] = (T)(i - degree_) / (numcontrol_points - degree_);
    }

    for (size_t i = numKnots - degree_ - 1; i < numKnots; ++i) {
        knotVector_[i] = 1.0;
    }
}

template<typename T>
const size_t CubicBSpline<T>::findKnotSpan(const size_t n, const T u)
{
    if (u >= knotVector_[n+1])
        return n;

    std::cout << "findKnotSpan n: " << n << " u: " << u << std::endl;
    std::cout << "knotVector_[n+1]: " << knotVector_[n+1] << std::endl;
    
    size_t low = degree_;
    size_t high = n + 1;
    size_t mid = (low + high) / 2;
    while (u < knotVector_[mid] || u >= knotVector_[mid+1]) {
        if (u < knotVector_[mid])
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// De Boor recursive function to evaluate basis functions
template<typename T>
const T CubicBSpline<T>::basisFunction(const size_t i, const size_t p, const T u)
{
    if (p == 0) {
        return (u >= knotVector_[i] && u < knotVector_[i+1]) ? 1.0 : 0.0;
    }

    double left = 0.0, right = 0.0;
    
    if (knotVector_[i + p] != knotVector_[i])
        left = (u - knotVector_[i]) / (knotVector_[i + p] - knotVector_[i]) * basisFunction(i, p - 1, u);
    
    if (knotVector_[i + p + 1] != knotVector_[i + 1])
        right = (knotVector_[i + p + 1] - u) / (knotVector_[i + p + 1] - knotVector_[i + 1]) * basisFunction(i + 1, p - 1, u);

    return left + right;
}

// Derivative of the basis function (1st and 2nd order)
template<typename T>
const T CubicBSpline<T>::basisFunctionDerivative(const size_t i, const size_t p, const T u, const size_t d_order)
{
    if (d_order == 0) {
        return basisFunction(i, p, u);
    }

    T left = 0.0, right = 0.0;

    if (knotVector_[i + p] != knotVector_[i]) {
        left = (p / (knotVector_[i + p] - knotVector_[i])) * basisFunctionDerivative(i, p - 1, u, d_order - 1);
    }

    if (knotVector_[i + p + 1] != knotVector_[i + 1]) {
        right = (p / (knotVector_[i + p + 1] - knotVector_[i + 1])) * basisFunctionDerivative(i + 1, p - 1, u, d_order - 1);
    }

    return left - right;
}

// Evaluate B-Spline or its derivatives at a given parameter u
template<typename T>
const std::array<T,2> CubicBSpline<T>::evaluate(const T u, const size_t d_order)
{
    const size_t n = control_points_.size() - 1;
    const auto span = findKnotSpan(n, u);

    std::cout << "spline eval with span: " << span << " | n: " << n << " | u: " << u << std::endl; 
    
    std::array<T,2> result = {0.0, 0.0};
    
    for (size_t i = 0; i <= degree_; ++i) {
        T coeff = (d_order == 0) ? basisFunction(span - degree_ + i, degree_, u) :
                        basisFunctionDerivative(span - degree_ + i, degree_, u, d_order);
        result[0] += coeff * control_points_[span - degree_ + i][0];
        result[1] += coeff * control_points_[span - degree_ + i][1];
    }
    
    return result;
}

// Evaluate B-Spline or its derivatives at given vector of parameters u
template<typename T>
const std::vector<std::array<T,2>> CubicBSpline<T>::evaluate(const std::vector<T> u_vec, const size_t d_order)
{
    std::vector<std::array<T,2>> result;
    result.reserve(u_vec.size());
    for (auto u : u_vec) {
        result.push_back(evaluate(u, d_order));
    }

    return result;
}

// Compute curvature from first and second derivatives
template<typename T>
const T CubicBSpline<T>::curvature(const T u) 
{
    const auto firstDerivative = evaluate(u, 1);
    const auto secondDerivative = evaluate(u, 2);
    T numerator = firstDerivative[0] * secondDerivative[1] - firstDerivative[1] * secondDerivative[0];
    T denominator = std::pow(firstDerivative[0] * firstDerivative[0] + firstDerivative[1] * firstDerivative[1], 1.5);
    
    return std::fabs(numerator) / denominator;
}

}// namespace nano_mppic::spline

typedef nano_mppic::spline::CubicBSpline<float> fCubicBSpline;
typedef nano_mppic::spline::CubicBSpline<double> dCubicBSpline;

#endif