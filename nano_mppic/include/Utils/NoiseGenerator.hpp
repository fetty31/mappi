#ifndef __NANO_MPPIC_NOISE_GENERATOR_HPP__
#define __NANO_MPPIC_NOISE_GENERATOR_HPP__

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace nano_mppic::utils {

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
        nano_mppic::config::Noise cfg_;

    // FUNCTIONS

    public:
        NoiseGenerator();

        void configure(nano_mppic::config::Noise cfg);

        void shutdown();

        void reset(nano_mppic::config::Noise cfg);

        void getControls(nano_mppic::objects::State& state, 
                        nano_mppic::objects::ControlSequence& ctrl_seq);

    private:

        void noiseThread();

        void generateNoise();

};

NoiseGenerator::NoiseGenerator() : active_(false), 
                                    ready_(false),
                                    is_holonomic_(false) { };

void NoiseGenerator::configure(nano_mppic::config::Noise cfg, bool is_holonomic)
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

void NoiseGenerator::reset(nano_mppic::config::Noise cfg)
{
    // Recompute the noises on reset
    {
        std::unique_lock<std::mutex> guard(noise_lock_);
        xt::noalias(noises_vx_) = xt::zeros<float>({cfg_.batch_size, cfg_.time_steps});
        xt::noalias(noises_vy_) = xt::zeros<float>({cfg_.batch_size, cfg_.time_steps});
        xt::noalias(noises_wz_) = xt::zeros<float>({cfg_.batch_size, cfg_.time_steps});
        ready_ = true;
    }
    noise_cond_.notify_all();
}

void NoiseGenerator::getControls(nano_mppic::objects::State& state, 
                                nano_mppic::objects::ControlSequence& ctrl_seq)
{
    std::unique_lock<std::mutex> guard(noise_lock_);

    xt::noalias(state.cvx) = ctrl_seq.vx + noises_vx_;
    xt::noalias(state.cvy) = ctrl_seq.vy + noises_vy_;
    xt::noalias(state.cwz) = ctrl_seq.wz + noises_wz_;

    {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
    }
    noise_cond_.notify_all();
}

void NoiseGenerator::noiseThread()
{
    do {
        std::unique_lock<std::mutex> guard(noise_lock_);
        noise_cond_.wait(guard, [this]() {return ready_;});
        ready_ = false;
        generateNoisedControls();
    } while (active_);
}

void NoiseGenerator::generateNoisedControls()
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

} // namespace nano_mppic::utils

#endif