#ifndef __NANO_MPPIC_STATE_HPP__
#define __NANO_MPPIC_STATE_HPP__

#include <xtensor/xtensor.hpp>

namespace nano_mppic::objects{

struct Odometry2d
{
    float x, y, yaw;
    float vx, vy, wz;
    float stamp;

    Odometry2d() : x(0.0), y(0.0), yaw(0.0),
                    vx(0.0), vy(0.0), wz(0.0), 
                    stamp(0.0) {}
}

struct State
{
  xt::xtensor<float, 2> vx;
  xt::xtensor<float, 2> vy;
  xt::xtensor<float, 2> wz;

  xt::xtensor<float, 2> cvx;
  xt::xtensor<float, 2> cvy;
  xt::xtensor<float, 2> cwz;

  Odometry2d odom;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = xt::zeros<float>({batch_size, time_steps});
    vy = xt::zeros<float>({batch_size, time_steps});
    wz = xt::zeros<float>({batch_size, time_steps});

    cvx = xt::zeros<float>({batch_size, time_steps});
    cvy = xt::zeros<float>({batch_size, time_steps});
    cwz = xt::zeros<float>({batch_size, time_steps});
  }
};


} // namespace nano_mppic::objects

#endif