/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   State and Odometry2D objects.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_STATE_HPP__
#define __MAPPI_STATE_HPP__

#include <xtensor/xtensor.hpp>

namespace mappi::objects{

struct Odometry2d
{
    double x, y, yaw;
    double vx, vy, wz;
    double steering;
    double stamp;

    /**
     * @brief Construct a new Odometry 2d object
     * 
     */
    Odometry2d() : x(0.0), y(0.0), yaw(0.0),
                    vx(0.0), vy(0.0), wz(0.0), 
                    steering(0.0), stamp(0.0) {}
};

struct State
{
  xt::xtensor<double, 2> vx;
  xt::xtensor<double, 2> vy;
  xt::xtensor<double, 2> wz;

  xt::xtensor<double, 2> cvx;
  xt::xtensor<double, 2> cvy;
  xt::xtensor<double, 2> cwz;

  Odometry2d odom;

  /**
   * @brief Resets the state
   * 
   * @param batch_size New batch dimension
   * @param time_steps New horizon length (prediction horizon)
   */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = xt::zeros<double>({batch_size, time_steps});
    vy = xt::zeros<double>({batch_size, time_steps});
    wz = xt::zeros<double>({batch_size, time_steps});

    cvx = xt::zeros<double>({batch_size, time_steps});
    cvy = xt::zeros<double>({batch_size, time_steps});
    cwz = xt::zeros<double>({batch_size, time_steps});
  }
};


} // namespace mappi::objects

#endif