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
                          std::array<mappi::objects::Control, 2> & ctrl_history)
{
  // Savitzky-Golay Quadratic, 5-point Coefficients
  xt::xarray<double> filter = {-3.0, 12.0, 17.0, 12.0, -3.0};
  filter /= 35.0;

  const unsigned int num_sequences = ctrl_seq.vx.shape(0);

  // Too short to smooth meaningfully
  if (num_sequences < 10) {
    return;
  }

  using xt::evaluation_strategy::immediate;
  auto applyFilter = [&](const xt::xarray<double> & data) -> double {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<double, 1> & sequence, const double hist_0, const double hist_1) -> void
    {
      unsigned int idx = 0;
      sequence(idx) = applyFilter(
      {
        hist_0,
        hist_1,
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_1,
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2)});

      for (idx = 2; idx != num_sequences - 3; idx++) {
        sequence(idx) = applyFilter(
        {
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2)});
      }

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 1)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx),
        sequence(idx)});
    };

  // Filter trajectories
  applyFilterOverAxis(ctrl_seq.vx, ctrl_history[0].vx, ctrl_history[1].vx);
  applyFilterOverAxis(ctrl_seq.vy, ctrl_history[0].vy, ctrl_history[1].vy);
  applyFilterOverAxis(ctrl_seq.wz, ctrl_history[0].wz, ctrl_history[1].wz);

  // Update control history
  static unsigned int offset = 1;
  ctrl_history[0] = ctrl_history[1];
  ctrl_history[1] = {
    ctrl_seq.vx(offset),
    ctrl_seq.vy(offset),
    ctrl_seq.wz(offset)};

}

/**
 * @brief Performs one step of the Quadratic Savitzky-Golay filter with window size of 9
 * 
 * @param ctrl_seq Control sequence to filter
 * @param ctrl_history Buffer of former controls
 */
void savitskyGolayFilter( mappi::objects::ControlSequence& ctrl_seq,
                          std::array<mappi::objects::Control, 4> & ctrl_history)
{
  // Savitzky-Golay Quadratic, 9-point Coefficients
  xt::xarray<double> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
  filter /= 231.0;

  const unsigned int num_sequences = ctrl_seq.vx.shape(0);

  // Too short to smooth meaningfully
  if (num_sequences < 20) {
    return;
  }

  using xt::evaluation_strategy::immediate;
  auto applyFilter = [&](const xt::xarray<double> & data) -> double {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<double, 1> & sequence, 
        const double hist_0, const double hist_1, 
        const double hist_2, const double hist_3) -> void
    {
      unsigned int idx = 0;
      sequence(idx) = applyFilter(
        {
          hist_0,
          hist_1,
          hist_2,
          hist_3,
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4) });

      idx++;
      sequence(idx) = applyFilter(
        {
          hist_1,
          hist_2,
          hist_3,
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
        {
          hist_2,
          hist_3,
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
        {
          hist_3,
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});

      for (idx = 4; idx != num_sequences - 5; idx++) {
        sequence(idx) = applyFilter(
          {
            sequence(idx - 4),
            sequence(idx - 3),
            sequence(idx - 2),
            sequence(idx - 1),
            sequence(idx),
            sequence(idx + 1),
            sequence(idx + 2),
            sequence(idx + 3),
            sequence(idx + 4)});
      }

      idx++;
      sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 3)});

      idx++;
      sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 2),
          sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 1),
          sequence(idx + 1),
          sequence(idx + 1)});

      idx++;
      sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx),
          sequence(idx),
          sequence(idx),
          sequence(idx)});
    };

  // Filter trajectories
  applyFilterOverAxis(ctrl_seq.vx, ctrl_history[0].vx, ctrl_history[1].vx, ctrl_history[2].vx, ctrl_history[3].vx);
  applyFilterOverAxis(ctrl_seq.vy, ctrl_history[0].vy, ctrl_history[1].vy, ctrl_history[2].vy, ctrl_history[3].vy);
  applyFilterOverAxis(ctrl_seq.wz, ctrl_history[0].wz, ctrl_history[1].wz, ctrl_history[2].wz, ctrl_history[3].wz);

  // Update control history
  static unsigned int offset = 1;
  ctrl_history[0] = ctrl_history[1];
  ctrl_history[1] = ctrl_history[2];
  ctrl_history[2] = ctrl_history[3];
  ctrl_history[3] = {
    ctrl_seq.vx(offset),
    ctrl_seq.vy(offset),
    ctrl_seq.wz(offset)};

}

} // namespace mappi::filters


#endif