#ifndef __MAPPI_TRAJECTORY_HPP__
#define __MAPPI_TRAJECTORY_HPP__

#include <xtensor/xtensor.hpp>

namespace mappi::objects {

struct Trajectory
{
    xt::xtensor<float, 2> x;
    xt::xtensor<float, 2> y;
    xt::xtensor<float, 2> yaw;

    /**
     * @brief Resets trajectory
     * 
     * @param batch_size New batch dimension
     * @param time_steps New horizon length (prediction horizon)
     */
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
    xt::xtensor<bool, 1> free;

    /**
     * @brief Resets path
     * 
     * @param size New path length
     */
    void reset(unsigned int size)
    {
        x = xt::zeros<float>({size});
        y = xt::zeros<float>({size});
        yaw = xt::zeros<float>({size});
        free = xt::ones<bool>({size});
    }

};

} // namespace mappi::objects

#endif