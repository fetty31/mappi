/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Auxiliar functions.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_AUX_HPP__
#define __MAPPI_AUX_HPP__

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "Objects/State.hpp"
#include "Objects/Trajectory.hpp"
#include "Objects/ControlSequence.hpp"

#include <cmath>

namespace mappi::aux {

/**
  * @brief Normalize an angular distance
  * 
  * @tparam T 
  * @param angles Angular distance [rad]
  * @return auto 
  */
template<typename T>
inline auto normalize_angles(const T & angles)
{
  // auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  auto && theta = xt::eval(xt::fmod( xt::fmod(angles, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI ));
  return xt::eval(xt::where(theta > M_PI, theta - 2.0*M_PI, theta));
}

/**
 * @brief Computes the shortest angular distance
 * 
 * @tparam F 
 * @tparam T 
 * @param from First angle
 * @param to Second angle
 * @return auto 
 */
template<typename F, typename T>
inline auto shortest_angular_dist(const F& from, const T& to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Computes L2 (Euclidean) norm
 * 
 * @tparam T 
 * @param p1 
 * @param p2 
 * @return T 
 */
template<typename T>
inline T l2_norm(const std::vector<T>& p1, const std::vector<T>& p2)
{
  if( (p1.size() < 2) || (p2.size() < 2) )
    return static_cast<T>(0.0);

  return std::sqrt( std::pow(p1[0]-p2[0],2) + std::pow(p1[1]-p2[1],2) );
}

/**
 * @brief Computes linspace (as in MATLAB)
 * 
 * @tparam T 
 * @param a Start
 * @param b End
 * @param N Number of equally spaced points
 * @return std::vector<T> 
 */
template <typename T>
inline std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

/**
 * @brief Computes the angle between a pose and a point
 * 
 * @param odom 2D Pose
 * @param x Point x coordinate
 * @param y Point y coordinate
 * @return double 
 */
inline double poseToPointAngle(mappi::objects::Odometry2d& odom, double x, double y)
{
  double diff_yaw = odom.yaw - std::atan2(y - odom.y, x - odom.x);
  double norm_yaw = std::fmod( std::fmod(diff_yaw, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI );

  return (norm_yaw > M_PI) ? norm_yaw - 2.0*M_PI : norm_yaw;
}

/**
 * @brief Finds the index of the furthest point along all the trajectories that is closer to the given path.
 * 
 * @param trajectories Predicted trajectories
 * @param plan Path to follow (plan)
 * @param time_step Point of the horizon length from where to start
 * @return size_t 
 */
inline size_t findPathMinDistPoint(const mappi::objects::Trajectory& trajectories,
                                    const mappi::objects::Path& plan,
                                    const int time_step = -1 )
{
  const auto traj_x = xt::view(trajectories.x, xt::all(), time_step, xt::newaxis());
  const auto traj_y = xt::view(trajectories.y, xt::all(), time_step, xt::newaxis());

  const auto dx = plan.x - traj_x; // xt::broadcast 
  const auto dy = plan.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0;
  double min_distance_by_path = std::numeric_limits<double>::max();

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

/**
 * @brief Computes whether the robot is near the goal. The goal is assumed to be the last point of the given plan.
 * 
 * @param pose_tolerance Tolerance to consider [m]
 * @param robot_pose Robot's position
 * @param plan Path to follow
 * @return true 
 * @return false 
 */
inline bool robotNearGoal(double pose_tolerance, 
                          const mappi::objects::Odometry2d& robot_pose,
                          const mappi::objects::Path& plan)
{
  const auto goal_idx = plan.x.shape(0)-1;
  const auto goal_x = plan.x(goal_idx);
  const auto goal_y = plan.y(goal_idx);

  const auto tolerance_sq = pose_tolerance*pose_tolerance;

  auto dx = robot_pose.x - goal_x;
  auto dy = robot_pose.y - goal_y;

  auto dist_sq = dx*dx + dy*dy;

  if(dist_sq < tolerance_sq)
    return true;
  else
    return false;

}

/**
 * @brief Looks for the index of first the point that is at a given distance from the starting point of a path 
 * 
 * @param path Path where to look
 * @param dist Distance
 * @return size_t 
 */
inline size_t getIdxFromDistance(mappi::objects::Path& path, double dist)
{
  if(path.x.size() < 1)
    return size_t(0);

  double cumdist = 0.0f;
  for(size_t i=0; i < path.x.size()-1; i++){
      cumdist += std::sqrt( std::pow(path.x(i+1)-path.x(i),2) + 
                            std::pow(path.y(i+1)-path.y(i),2) );
      if(cumdist >= dist)
          return i;
  }

  return path.x.size()-1;
}

/**
 * @brief Shift a given path by some distance
 * 
 * @param path Path to shift
 * @param dist Distance to shift
 */
inline void shiftPlan(mappi::objects::Path& path, double dist)
{
  size_t index = aux::getIdxFromDistance(path, dist);

  if(index > path.x.size()-5)
    return;

  if(index < 1)
    return;

  mappi::objects::Path path_cpy = path;

  // shift path
  path_cpy.x    = xt::roll(path_cpy.x,    -index);
  path_cpy.y    = xt::roll(path_cpy.y,    -index);
  path_cpy.yaw  = xt::roll(path_cpy.yaw,  -index);
  path_cpy.free = xt::roll(path_cpy.free, -index);

  // reset path shape
  path.reset(path_cpy.x.size()-index);

  // copy shifted path 
  xt::view(path.x,    xt::all() ) = xt::view(path_cpy.x,    xt::range(xt::placeholders::_, path.x.size()));
  xt::view(path.y,    xt::all() ) = xt::view(path_cpy.y,    xt::range(xt::placeholders::_, path.y.size()));
  xt::view(path.yaw,  xt::all() ) = xt::view(path_cpy.yaw,  xt::range(xt::placeholders::_, path.yaw.size()));
  xt::view(path.free, xt::all() ) = xt::view(path_cpy.free, xt::range(xt::placeholders::_, path.free.size()));
}

} // namespace mappi::aux


#endif