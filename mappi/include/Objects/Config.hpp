#ifndef __MAPPI_CONFIG_HPP__
#define __MAPPI_CONFIG_HPP__

#include <string>

namespace mappi::config {

struct Noise
{
    unsigned int batch_size;
    unsigned int time_steps;

    float std_vx, std_vy, std_wz;
};

struct AckermannModel 
{
    float min_r;
};

struct BicycleKinModel 
{
    float length;
};

struct Constraints 
{
    float max_vx, min_vx;
    float max_vy, min_vy;
    float max_wz, min_wz;
};

struct CommonCost
{
    bool active;
    unsigned int power;
    float weight;
    float threshold;
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
    size_t offset_from_furthest;
};

struct PathAngleCritic
{
    CommonCost common;
    size_t offset_from_furthest;
    float angle_threshold;
};

struct ObstaclesCritic
{
    CommonCost common;
    float repulsive_weight;
    float inflation_radius;
    float inflation_scale_factor;
    float collision_cost;
    float collision_margin_dist;
};

struct MPPIc 
{
    struct Settings
    {
        unsigned int num_iters;
        unsigned int num_retry;
        unsigned int offset;
        bool use_splines;
        std::string motion_model;
    } settings;

    AckermannModel ackermann;
    BicycleKinModel steering;
    Noise noise;
    Constraints bounds;

    PathCritic pathfollow_crtc;
    PathAngleCritic pathangle_crtc;
    GenericCritic goal_crtc;
    PathDistCritic pathdist_crtc;
    GenericCritic twir_crtc;
    GenericCritic goalangle_crtc;
    ObstaclesCritic obs_crtc;

    float model_dt;
    float temperature;
    float gamma;

    void print_out();
};

void MPPIc::print_out(){
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

    std::cout << "MotionModel: " << settings.motion_model << std::endl;
    if (settings.motion_model=="Ackermann")
        std::cout << "  - min_radius: " << ackermann.min_r << std::endl;
    else if (settings.motion_model=="BicycleKin")
        std::cout << "  - length: " << steering.length << std::endl;

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

} // namespace mappi::config

#endif