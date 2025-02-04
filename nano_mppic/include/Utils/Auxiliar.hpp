#ifndef __NANO_MPPI_AUX_HPP__
#define __NANO_MPPI_AUX_HPP__

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "Objects/State.hpp"
#include "Objects/Trajectory.hpp"
#include "Objects/ControlSequence.hpp"

#include <cmath>

namespace nano_mppic::aux {

template<typename T>
auto normalize_angles(const T & angles)
{
  // auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  auto && theta = xt::eval(xt::fmod( xt::fmod(angles, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI ));
  return xt::eval(xt::where(theta > M_PI, theta - 2.0*M_PI, theta));
}

template<typename F, typename T>
auto shortest_angular_dist(const F& from, const T& to)
{
  return normalize_angles(to - from);
}

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

inline float poseToPointAngle(nano_mppic::objects::Odometry2d& odom, float x, float y)
{
  float diff_yaw = odom.yaw - std::atan2(y - odom.y, x - odom.x);
  float norm_yaw = std::fmod( std::fmod(diff_yaw, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI );

  return (norm_yaw > M_PI) ? norm_yaw - 2.0*M_PI : norm_yaw;
}

size_t findPathMinDistPoint(const nano_mppic::objects::Trajectory& trajectories,
                            const nano_mppic::objects::Path& plan,
                            const int time_step = -1 )
{
  const auto traj_x = xt::view(trajectories.x, xt::all(), time_step, xt::newaxis());
  const auto traj_y = xt::view(trajectories.y, xt::all(), time_step, xt::newaxis());

  const auto dx = plan.x - traj_x; // xt::broadcast 
  const auto dy = plan.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();

  for (size_t i = 0; i < dists.shape(0); i++) {
    size_t min_id_by_path = 0;
    for (size_t j = 0; j < dists.shape(1); j++) {
      if (dists(i, j) < min_distance_by_path) {
        min_distance_by_path = dists(i, j);
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }

  return max_id_by_trajectories;
}

bool robotNearGoal(float pose_tolerance, 
                    const nano_mppic::objects::Odometry2d& robot_pose,
                    const nano_mppic::objects::Path& plan)
{
  const auto goal_idx = plan.x.shape(0)-1;
  const auto goal_x = plan.x(goal_idx);
  const auto goal_y = plan.y(goal_idx);

  const auto tolerance_sq = pose_tolerance*pose_tolerance;

  auto dx = robot_pose.x - goal_x;
  auto dy = robot_pose.y - goal_y;

  auto dist_sq = dx*dx + dy*dy;

  if(dist_sq < pose_tolerance)
    return true;
  else
    return false;

}

size_t getIdxFromDistance(nano_mppic::objects::Path& path, float dist)
{
  float cumdist = 0.0f;
  for(size_t i=0; i < path.x.size()-1; i++){
      cumdist += std::sqrt( std::pow(path.x(i+1)-path.x(i),2) + 
                            std::pow(path.y(i+1)-path.y(i),2) );
      if(cumdist >= dist)
          return i;
  }

  return path.x.size();
}

void savitskyGolayFilter( nano_mppic::objects::ControlSequence& ctrl_seq,
                          std::array<nano_mppic::objects::Control, 2> & ctrl_history)
{
  // Savitzky-Golay Quadratic, 5-point Coefficients
  xt::xarray<float> filter = {-3.0, 12.0, 17.0, 12.0, -3.0};
  filter /= 35.0;

  const unsigned int num_sequences = ctrl_seq.vx.shape(0);

  // Too short to smooth meaningfully
  if (num_sequences < 10) {
    return;
  }

  using xt::evaluation_strategy::immediate;
  auto applyFilter = [&](const xt::xarray<float> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence, const float hist_0, const float hist_1) -> void
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
  unsigned int offset = 1;
  ctrl_history[0] = ctrl_history[1];
  ctrl_history[1] = {
    ctrl_seq.vx(offset),
    ctrl_seq.vy(offset),
    ctrl_seq.wz(offset)};

}

} // namespace nano_mppic::aux


#endif