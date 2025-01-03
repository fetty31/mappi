#ifndef __NANO_MPPIC_CONFIG_HPP__
#define __NANO_MPPIC_CONFIG_HPP__

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
}

} // namespace nano_mppic::config

#endif