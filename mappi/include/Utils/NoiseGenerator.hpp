/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Noise Generator class. It computes randomly (Gaussian) distributed control inputs in a seperate thread.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_NOISE_GENERATOR_HPP__
#define __MAPPI_NOISE_GENERATOR_HPP__

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "Objects/Config.hpp"
#include "Objects/State.hpp"
#include "Objects/ControlSequence.hpp"

namespace mappi::utils {

class NoiseGenerator {

    // VARIABLES

    private:
        xt::xtensor<double, 2> noises_vx_;
        xt::xtensor<double, 2> noises_vy_;
        xt::xtensor<double, 2> noises_wz_;

        std::thread noise_thread_;
        std::condition_variable noise_cond_;
        std::mutex noise_lock_;
        bool active_, ready_;

        bool is_holonomic_;
        mappi::config::Noise cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Noise Generator object
         * 
         */
        NoiseGenerator();

        /**
         * @brief Configure the Noise Generator
         * 
         * @param cfg Configuration struct
         * @param is_holonomic Whether the used motion model is holonomic. 
         */
        void configure(mappi::config::Noise& cfg, bool is_holonomic);

        /**
         * @brief Stop the noise generating thread
         * 
         */
        void shutdown();

        /**
         * @brief Reset the Noise Generator
         * 
         * @param cfg New configuration struct
         * @param is_holonomic New holonomic flag
         */
        void reset(mappi::config::Noise cfg, bool is_holonomic);

        /**
         * @brief Get the newly computed controls
         * 
         * @param state Current state
         * @param ctrl_seq Control sequence
         */
        void getControls(mappi::objects::State& state, 
                        mappi::objects::ControlSequence& ctrl_seq);

    private:
        
        /**
         * @brief Start the noise generating thread
         * 
         */
        void noiseThread();
        
        /**
         * @brief Generate control noise
         * 
         */
        void generateNoise();

};

} // namespace mappi::utils

#endif