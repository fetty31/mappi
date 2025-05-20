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

namespace mappi::utils {

class NoiseGenerator {

    // VARIABLES

    private:
        xt::xtensor<float, 2> noises_vx_;
        xt::xtensor<float, 2> noises_vy_;
        xt::xtensor<float, 2> noises_wz_;

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

NoiseGenerator::NoiseGenerator() : active_(false), 
                                    ready_(false),
                                    is_holonomic_(false) { };

void NoiseGenerator::configure(mappi::config::Noise& cfg, bool is_holonomic)
{
    cfg_ = cfg;
    is_holonomic_ = is_holonomic;
    active_ = ready_ = true;
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
}

void NoiseGenerator::shutdown()
{
    active_ = false;
    ready_ = true;
    noise_cond_.notify_all();
    if (noise_thread_.joinable())
        noise_thread_.join();
}

void NoiseGenerator::reset(mappi::config::Noise cfg, bool is_holonomic)
{
    // Recompute the noises on reset
    std::unique_lock<std::mutex> guard(noise_lock_);
    cfg_ = cfg;
    xt::noalias(noises_vx_) = xt::zeros<float>({cfg_.batch_size, cfg_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({cfg_.batch_size, cfg_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({cfg_.batch_size, cfg_.time_steps});
    is_holonomic_ = is_holonomic;

    ready_ = true;
    noise_cond_.notify_all();
}

void NoiseGenerator::getControls(mappi::objects::State& state, 
                                mappi::objects::ControlSequence& ctrl_seq)
{
    std::unique_lock<std::mutex> guard(noise_lock_);

    xt::noalias(state.cvx) = ctrl_seq.vx + noises_vx_;
    xt::noalias(state.cvy) = ctrl_seq.vy + noises_vy_;
    xt::noalias(state.cwz) = ctrl_seq.wz + noises_wz_;

    ready_ = true;
    noise_cond_.notify_all();
}

void NoiseGenerator::noiseThread()
{
    do {
        std::unique_lock<std::mutex> guard(noise_lock_);
        noise_cond_.wait(guard, [this]() {return ready_;});
        generateNoise();
        ready_ = false;
    } while (active_);
}

void NoiseGenerator::generateNoise()
{
    xt::noalias(noises_vx_) = xt::random::randn<float>(
        {cfg_.batch_size, cfg_.time_steps}, 0.0, cfg_.std_vx);
    xt::noalias(noises_wz_) = xt::random::randn<float>(
        {cfg_.batch_size, cfg_.time_steps}, 0.0, cfg_.std_wz);
    if (is_holonomic_) {
        xt::noalias(noises_vy_) = xt::random::randn<float>(
            {cfg_.batch_size, cfg_.time_steps}, 0.0, cfg_.std_vy);
    }
}

} // namespace mappi::utils

#endif