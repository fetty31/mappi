#ifndef __NANO_MPPIC_CONTROL_SEQUENCE_HPP__
#define __NANO_MPPIC_CONTROL_SEQUENCE_HPP__

#include <xtensor/xtensor.hpp>

namespace nano_mppic::objects {

struct Control
{
  float vx, vy, wz;
};

struct ControlSequence
{
  xt::xtensor<float, 1> vx;
  xt::xtensor<float, 1> vy;
  xt::xtensor<float, 1> wz;

  void reset(unsigned int time_steps)
  {
    vx = xt::zeros<float>({time_steps});
    vy = xt::zeros<float>({time_steps});
    wz = xt::zeros<float>({time_steps});
  }
};

} // namespace nano_mppic::objects

#endif