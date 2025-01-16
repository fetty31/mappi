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

    void print_out();
};

void MPPIc::print_out(){
    std::cout << "//////////////// MPPIc::Configuration ////////////////\n";
    std::cout << "num_iters: " << settings.num_iters << std::endl;
    std::cout << "num_retry: " << settings.num_retry << std::endl;
    std::cout << "batch_size: " << noise.batch_size << std::endl;
    std::cout << "time_steps: " << noise.time_steps << std::endl;
    std::cout << "offset: " << settings.offset << std::endl;
    std::cout << "model_dt: " << model_dt << std::endl;
    std::cout << "temperature: " << temperature  << std::endl;
    std::cout << "gamma: " << gamma << std::endl;
    std::cout << "motion_model: " << settings.motion_model << std::endl;
    if (settings.motion_model=="Ackermann")
        std::cout << "ackermann.min_r: " << ackermann.min_r << std::endl;
    std::cout << "noise.std_vx: " << noise.std_vx << std::endl;
    std::cout << "noise.std_vy: " << noise.std_vy << std::endl;
    std::cout << "noise.std_wz: " << noise.std_wz << std::endl;
    std::cout << "bounds.max_vx: " << bounds.max_vx << "   bounds.min_vx: " << bounds.min_vx << std::endl;
    std::cout << "bounds.max_vy: " << bounds.max_vy << "   bounds.min_vy: " << bounds.min_vy << std::endl;
    std::cout << "bounds.max_wz: " << bounds.max_wz << "   bounds.min_wz: " << bounds.min_wz 
    << "//////////////////////////////////////////////////////\n" <<
    std::endl;
}

} // namespace nano_mppic::config

#endif