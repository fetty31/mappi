/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Control Sequence and Control objects.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_CONTROL_SEQUENCE_HPP__
#define __MAPPI_CONTROL_SEQUENCE_HPP__

#include <xtensor/xtensor.hpp>

namespace mappi::objects {

struct Control
{
  float vx{0.0f}, vy{0.0f}, wz{0.0f};
};

struct ControlSequence
{
  xt::xtensor<float, 1> vx;
  xt::xtensor<float, 1> vy;
  xt::xtensor<float, 1> wz;

  /**
   * @brief Resets the control sequence
   * 
   * @param time_steps New horizon length (prediction horizon)
   */
  void reset(unsigned int time_steps)
  {
    vx = xt::zeros<float>({time_steps});
    vy = xt::zeros<float>({time_steps});
    wz = xt::zeros<float>({time_steps});
  }
};

} // namespace mappi::objects

#endif