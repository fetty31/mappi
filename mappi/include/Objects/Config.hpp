/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Configuration objects.
 *      types matching ROS2 parameters types: {double, int, string, bool}
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_CONFIG_HPP__
#define __MAPPI_CONFIG_HPP__

#include <string>
#include <iostream>

namespace mappi::config {

struct Noise
{
    int batch_size;
    int time_steps;

    double std_vx, std_vy, std_wz;
};

struct AckermannModel 
{
    double min_r;
};

struct BicycleKinModel 
{
    double length;
    double max_steer;
};

struct Constraints 
{
    double max_vx, min_vx;
    double max_vy, min_vy;
    double max_wz, min_wz;
};

struct CommonCost
{
    bool active;
    int power;
    double weight;
    double threshold;
};

struct GenericCritic{
    CommonCost common;
};

struct PathDistCritic{
    CommonCost common;
    int traj_stride;
};

struct PathCritic 
{
    CommonCost common;
    int offset_from_furthest;
};

struct PathAngleCritic
{
    CommonCost common;
    int offset_from_furthest;
    double angle_threshold;
};

struct ObstaclesCritic
{
    CommonCost common;
    double repulsive_weight;
    double inflation_radius;
    double inflation_scale_factor;
    double collision_cost;
    double collision_margin_dist;
};

struct Visualization
{
    bool active;
    int batch_stride;
    int time_stride;
    double default_z;
    double scale;
};

struct MPPIc 
{
    struct Settings
    {
        int num_iters;
        int num_retry;
        int offset;
        double goal_tolerance;
        double dist_shift;
        bool use_splines;
        std::string motion_model;
        std::string global_frame;
        std::string local_frame;
    } settings;

    AckermannModel ackermann;
    BicycleKinModel bicycleKin;
    Noise noise;
    Constraints bounds;

    PathCritic pathfollow_crtc;
    PathAngleCritic pathangle_crtc;
    GenericCritic goal_crtc;
    PathDistCritic pathdist_crtc;
    GenericCritic twir_crtc;
    GenericCritic forward_crtc;
    GenericCritic goalangle_crtc;
    ObstaclesCritic obs_crtc;

    Visualization visual;

    double model_dt;
    double temperature;
    double gamma;

    /**
     * @brief Aux. function to print out all set parameters
     * 
     */
    void print_out() {
        std::cout << "\n//////////////// MPPIc::Configuration ////////////////\n";
        std::cout << "General Settings: " << std::endl;
        std::cout << "  - num_iters: " << settings.num_iters << std::endl;
        std::cout << "  - num_retry: " << settings.num_retry << std::endl;
        std::cout << "  - batch_size: " << noise.batch_size << std::endl;
        std::cout << "  - time_steps: " << noise.time_steps << std::endl;
        std::cout << "  - offset: " << settings.offset << std::endl;
        std::cout << "  - model_dt: " << model_dt << std::endl;
        std::cout << "  - temperature: " << temperature  << std::endl;
        std::cout << "  - gamma: " << gamma << std::endl;
        std::cout << "  - use_splines: " << settings.use_splines << std::endl;
        std::cout << "  - dist_shift: " << settings.dist_shift << std::endl;
        std::cout << "  - goal_tolerance: " << settings.goal_tolerance << std::endl;
        std::cout << "  - global_frame: " << settings.global_frame << std::endl;
        std::cout << "  - local_frame: " << settings.local_frame << std::endl;

        std::cout << "MotionModel: " << settings.motion_model << std::endl;
        if (settings.motion_model=="Ackermann")
            std::cout << "  - min_radius: " << ackermann.min_r << std::endl;
        else if (settings.motion_model=="BicycleKin"){
            std::cout << "  - length: " << bicycleKin.length << std::endl;
            std::cout << "  - max steering: " << bicycleKin.max_steer << std::endl;
        }

        std::cout << "Visualization: " << std::endl;
        std::cout << "  - visual.active: "          << visual.active        << std::endl;
        std::cout << "  - visual.batch_stride: "    << visual.batch_stride  << std::endl;
        std::cout << "  - visual.time_stride: "     << visual.time_stride   << std::endl;
        std::cout << "  - visual.default_z: "       << visual.default_z     << std::endl;
        std::cout << "  - visual.scale: "           << visual.scale         << std::endl;

        std::cout << "Noise Settings: " << std::endl;
        std::cout << "  - noise.std_vx: " << noise.std_vx << std::endl;
        std::cout << "  - noise.std_vy: " << noise.std_vy << std::endl;
        std::cout << "  - noise.std_wz: " << noise.std_wz << std::endl;

        std::cout << "Constraints Settings: " << std::endl;
        std::cout << "  - bounds.max_vx: " << bounds.max_vx << "   bounds.min_vx: " << bounds.min_vx << std::endl;
        std::cout << "  - bounds.max_vy: " << bounds.max_vy << "   bounds.min_vy: " << bounds.min_vy << std::endl;
        std::cout << "  - bounds.max_wz: " << bounds.max_wz << "   bounds.min_wz: " << bounds.min_wz << std::endl;

        std::cout << "Goal Critic Settings: " << std::endl;
        std::cout << "  - active: "     << goal_crtc.common.active       << std::endl;
        std::cout << "  - power: "      << goal_crtc.common.power       << std::endl;
        std::cout << "  - weight: "     << goal_crtc.common.weight      << std::endl;
        std::cout << "  - threshold: "  << goal_crtc.common.threshold   << std::endl;

        std::cout << "Path Distance Settings: " << std::endl;
        std::cout << "  - active: "     << pathdist_crtc.common.active       << std::endl;
        std::cout << "  - power: "      << pathdist_crtc.common.power       << std::endl;
        std::cout << "  - weight: "     << pathdist_crtc.common.weight      << std::endl;
        std::cout << "  - threshold: "     << pathdist_crtc.common.threshold      << std::endl;
        std::cout << "  - stride: "     << pathdist_crtc.traj_stride      << std::endl;

        std::cout << "Twirling Settings: " << std::endl;
        std::cout << "  - active: "     << twir_crtc.common.active       << std::endl;
        std::cout << "  - power: "      << twir_crtc.common.power       << std::endl;
        std::cout << "  - weight: "     << twir_crtc.common.weight      << std::endl;

        std::cout << "Path Follow Critic Settings: " << std::endl;
        std::cout << "  - active: "     << pathfollow_crtc.common.active       << std::endl;
        std::cout << "  - power: "      << pathfollow_crtc.common.power       << std::endl;
        std::cout << "  - weight: "     << pathfollow_crtc.common.weight      << std::endl;
        std::cout << "  - threshold: "  << pathfollow_crtc.common.threshold   << std::endl;
        std::cout << "  - offset_from_furthest: "  << pathfollow_crtc.offset_from_furthest  << std::endl;

        std::cout << "Path Angle Critic Settings: " << std::endl;
        std::cout << "  - active: "     << pathangle_crtc.common.active       << std::endl;
        std::cout << "  - power: "      << pathangle_crtc.common.power       << std::endl;
        std::cout << "  - weight: "     << pathangle_crtc.common.weight      << std::endl;
        std::cout << "  - threshold: "  << pathangle_crtc.common.threshold   << std::endl;
        std::cout << "  - offset_from_furthest: "  << pathangle_crtc.offset_from_furthest  << std::endl;
        std::cout << "  - angle_threshold: "  << pathangle_crtc.angle_threshold  << std::endl;

        std::cout << "Obstacles Critic Settings: " << std::endl;
        std::cout << "  - active: "     << obs_crtc.common.active      << std::endl;
        std::cout << "  - power: "      << obs_crtc.common.power       << std::endl;
        std::cout << "  - weight: "     << obs_crtc.common.weight      << std::endl;
        std::cout << "  - threshold: "  << obs_crtc.common.threshold   << std::endl;
        std::cout << "  - repulsive weight: "       << obs_crtc.repulsive_weight        << std::endl;
        std::cout << "  - inflation_radius: "       << obs_crtc.inflation_radius        << std::endl;
        std::cout << "  - inflation_scale_factor: " << obs_crtc.inflation_scale_factor  << std::endl;
        std::cout << "  - collision_cost: "         << obs_crtc.collision_cost          << std::endl;
        std::cout << "  - collision_margin_dist: "  << obs_crtc.collision_margin_dist 

        << "\n//////////////////////////////////////////////////////\n" <<
        std::endl;
    }
};

} // namespace mappi::config

#endif