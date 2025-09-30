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
         * @return size_t 
         */
        size_t findKnotSpan(const size_t n, const T u);

        /**
         * @brief B-spline basis function
         * 
         * @param i 
         * @param p 
         * @param u 
         * @return const T 
         */
        const T basisFunction(const size_t i, const size_t p, const T u);
        
        /**
         * @brief B-spline derivative basis function
         * 
         * @param i 
         * @param p 
         * @param u 
         * @param d_order 
         * @return const T 
         */
        const T basisFunctionDerivative(const size_t i, const size_t p, const T u, const size_t d_order);

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
        const std::vector<T> evaluate(const T u, const size_t d_order) override;

        /**
         * @brief Evaluate spline at multiple points
         * 
         * @param u_vec Points where to eval
         * @param d_order Derivative order
         * @return const std::vector<std::vector<T>> 
         */
        const std::vector<std::vector<T>> evaluate(const std::vector<T> u_vec, const size_t d_order) override;
};

template<typename T>
CubicBSpline<T>::CubicBSpline() : BaseSpline<T>() { }

template<typename T>
CubicBSpline<T>::CubicBSpline(const std::vector<std::vector<T>>& control_points)
                                                : BaseSpline<T>(control_points) 
{ 
    // initialize();
}

template<typename T>
void CubicBSpline<T>::initialize()
{
    const size_t numcontrol_points = this->control_points_.size();
    const size_t numKnots = numcontrol_points + this->degree_ + 1;
    knotVector_.reserve(numKnots);

    for (size_t i = 0; i <= this->degree_; ++i) {
        knotVector_[i] = 0.0;
    }

    for (size_t i = this->degree_ + 1; i < numKnots - this->degree_ - 1; ++i) {
        knotVector_[i] = (T)(i - this->degree_) / (numcontrol_points - this->degree_);
    }

    for (size_t i = numKnots - this->degree_ - 1; i < numKnots; ++i) {
        knotVector_[i] = 1.0;
    }
}

template<typename T>
size_t CubicBSpline<T>::findKnotSpan(const size_t n, const T u)
{
    if (u >= knotVector_[n+1])
        return n;

    size_t low = this->degree_;
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
const std::vector<T> CubicBSpline<T>::evaluate(const T u, const size_t d_order)
{
    const size_t n = this->control_points_.size() - 1;
    const auto span = findKnotSpan(n, u);

    const size_t interp_n = this->control_points_[0].size();
    std::vector<T> result(interp_n, static_cast<T>(0.0));
    
    for (size_t i = 0; i <= this->degree_; ++i) {
        T coeff = (d_order == 0) ? basisFunction(span - this->degree_ + i, this->degree_, u) :
                        basisFunctionDerivative(span - this->degree_ + i, this->degree_, u, d_order);

        for(size_t j = 0; j < interp_n; j++)
            result[j] += coeff * this->control_points_[span - this->degree_ + i][j];
    }
    
    return result;
}

// Evaluate B-Spline or its derivatives at given vector of parameters u
template<typename T>
const std::vector<std::vector<T>> CubicBSpline<T>::evaluate(const std::vector<T> u_vec, const size_t d_order)
{
    std::vector<std::vector<T>> result;
    result.reserve(u_vec.size());
    for (auto u : u_vec) {
        result.push_back(evaluate(u, d_order));
    }

    return result;
}

}// namespace mappi::spline

typedef mappi::spline::CubicBSpline<float> fCubicBSpline;
typedef mappi::spline::CubicBSpline<double> dCubicBSpline;

#endif