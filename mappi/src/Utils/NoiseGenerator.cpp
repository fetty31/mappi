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
 
#include "Utils/NoiseGenerator.hpp"

namespace mappi::utils {

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
    xt::noalias(noises_vx_) = xt::zeros<double>({cfg_.batch_size, cfg_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<double>({cfg_.batch_size, cfg_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<double>({cfg_.batch_size, cfg_.time_steps});
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
    xt::noalias(noises_vx_) = xt::random::randn<double>(
        {cfg_.batch_size, cfg_.time_steps}, 0.0, cfg_.std_vx);
    xt::noalias(noises_wz_) = xt::random::randn<double>(
        {cfg_.batch_size, cfg_.time_steps}, 0.0, cfg_.std_wz);
    if (is_holonomic_) {
        xt::noalias(noises_vy_) = xt::random::randn<double>(
            {cfg_.batch_size, cfg_.time_steps}, 0.0, cfg_.std_vy);
    }
}

} // namespace mappi::utils