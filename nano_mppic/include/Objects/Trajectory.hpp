#ifndef __NANO_MPPI_TRAJECTORY_HPP__
#define __NANO_MPPI_TRAJECTORY_HPP__

#include <xtensor/xtensor.hpp>

namespace nano_mppic::objects {

struct Trajectory
{
    xt::xtensor<float, 2> x;
    xt::xtensor<float, 2> y;
    xt::xtensor<float, 2> yaw;

    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaw = xt::zeros<float>({batch_size, time_steps});
    }

};

struct Path
{
    xt::xtensor<float, 1> x;
    xt::xtensor<float, 1> y;
    xt::xtensor<float, 1> yaw;

    void reset(unsigned int size)
    {
        x = xt::zeros<float>({size});
        y = xt::zeros<float>({size});
        yaw = xt::zeros<float>({size});
    }

};

} // namespace nano_mppic::objects

#endif