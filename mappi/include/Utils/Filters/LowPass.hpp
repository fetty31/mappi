/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Low Pass Filtering functions.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_LOW_PASS_FILTERS_HPP__
#define __MAPPI_LOW_PASS_FILTERS_HPP__

#include <cmath>
#include <chrono>

namespace mappi::filters {

/**
 * @brief Computes one step of low pass filter
 * 
 * @tparam T 
 * @param input 
 * @return T 
 */
template<typename T>
T lowPassFilter(T input);

/**
 * @brief Computes one step of low pass filter on 3D signal
 * 
 * @tparam T 
 * @param input_0 
 * @param input_1 
 * @param input_2 
 * @param output_0 
 * @param output_1 
 * @param output_2 
 */
template<typename T>
void lowPassFilter(T input_0, T input_1, T input_2,
                  T& output_0, T& output_1, T& output_2);

} // namespace mappi::filters


#endif