/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Savitsky Golay Filtering functions.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_SAVITSKY_FILTERS_HPP__
#define __MAPPI_SAVITSKY_FILTERS_HPP__

#include <xtensor/xarray.hpp>
#include <xtensor/xmanipulation.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "Objects/Trajectory.hpp"
#include "Objects/ControlSequence.hpp"

#include <cmath>
#include <chrono>

namespace mappi::filters {

/**
* @brief Performs one step of the Quadratic Savitzky-Golay filter with window size of 5
* 
* @param ctrl_seq Control sequence to filter
* @param ctrl_history Buffer of former controls
*/
void savitskyGolayFilter( mappi::objects::ControlSequence& ctrl_seq,
                          std::array<mappi::objects::Control, 2> & ctrl_history);

/**
 * @brief Performs one step of the Quadratic Savitzky-Golay filter with window size of 9
 * 
 * @param ctrl_seq Control sequence to filter
 * @param ctrl_history Buffer of former controls
 */
void savitskyGolayFilter( mappi::objects::ControlSequence& ctrl_seq,
                          std::array<mappi::objects::Control, 4> & ctrl_history);

} // namespace mappi::filters


#endif