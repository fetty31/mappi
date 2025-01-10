#ifndef __NANO_MPPIC_CONFIG_HPP__
#define __NANO_MPPIC_CONFIG_HPP__

#include <string>

/*To-Do:
    - Define configure structures for
        . Controller
        . Prediction
        . Noise
        . Models constraints
        . Critics
*/

namespace nano_mppic::config {

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

struct Constraints 
{
    float max_vx, min_vx;
    float max_vy, min_vy;
    float max_wz, min_wz;
};

struct MPPIc 
{
    struct Settings
    {
        unsigned int num_iters;
        unsigned int batch_size;
        unsigned int time_steps;
        unsigned int num_retry;
        unsigned int offset;
        std::string motion_model;
    } settings;

    AckermannModel ackermann;
    Noise noise;
    Constraints bounds;

    float model_dt;
    float temperature;
    float gamma;
};

} // namespace nano_mppic::config

#endif