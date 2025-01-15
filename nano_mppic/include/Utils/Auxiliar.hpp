#ifndef __NANO_MPPI_AUX_HPP__
#define __NANO_MPPI_AUX_HPP__

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

namespace nano_mppic::aux {

template<typename T>
auto normalize_angles(const T & angles)
{
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}

size_t findPathMinDistPoint(const nano_mppic::objects::Trajectory& trajectories,
                            const nano_mppic::objects::Path& plan)
{
  const auto traj_x = xt::view(trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(trajectories.y, xt::all(), -1, xt::newaxis());

  const auto dx = plan.x - traj_x;
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

} // namespace nano_mppic::aux


#endif