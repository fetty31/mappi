#include "mppic.hpp"

namespace nano_mppic {

using namespace xt::placeholders;
using xt::evaluation_strategy::immediate;

MPPIc::MPPIc() : is_configured_(false) { }
        
void MPPIc::configure(config::MPPIc& cfg, 
                            nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
    cfg_ = cfg;
    
    // depending on cfg.settings.motion_model -> declare model
    if(cfg.settings.motion_model == "Ackermann")
        motion_mdl_ptr_ = std::make_unique<models::Ackermann>(cfg_.ackermann, cfg_.model_dt);
    else
        motion_mdl_ptr_ = std::make_unique<models::Holonomic>(cfg_.model_dt);

    noise_gen_.configure(cfg.noise, isHolonomic());

    obs_critic_.configure("obstacles_critic", cfg.obs_crtc, costmap);
    goal_critic_.configure("goal_critic", cfg.goal_crtc, costmap);
    pathfollow_critic_.configure("pathfollow_critic", cfg.pathfollow_crtc, costmap);
    pathdist_critic_.configure("pathdist_critic", cfg.pathdist_crtc, costmap);
    goalangle_critic_.configure("goalangle_critic", cfg.goalangle_crtc, costmap);
    twir_critic_.configure("twir_critic", cfg.twir_crtc, costmap);

    this->reset(); // quick set up of obj dimensions

    is_configured_ = true;
}

void MPPIc::shutdown()
{
    noise_gen_.shutdown(); // join noise threads
}

void MPPIc::reset()
{
    state_.reset(cfg_.noise.batch_size, cfg_.noise.time_steps);
    trajectory_.reset(cfg_.noise.batch_size, cfg_.noise.time_steps);
    ctrl_seq_.reset(cfg_.noise.time_steps);

    noise_gen_.reset(cfg_.noise, isHolonomic());

    costs_ = xt::zeros<float>({cfg_.noise.batch_size});
}

void MPPIc::setConfig(config::MPPIc& config)
{

    std::unique_lock<std::mutex> guard(p_lock_); // lock for param update

    std::cout << "NANO_MPPIC::MPPIc updating parameters...\n";

    if(config.settings.motion_model == "Ackermann")
        motion_mdl_ptr_ = std::make_unique<models::Ackermann>(config.ackermann, config.model_dt);
    else
        motion_mdl_ptr_ = std::make_unique<models::Holonomic>(config.model_dt);

    if( (config.noise.batch_size != cfg_.noise.batch_size) || 
        (config.noise.time_steps != cfg_.noise.time_steps) )
    {
        cfg_ = config;
        reset(); // reset full MPPIc
    }
    else
    {
        cfg_ = config;
        noise_gen_.reset(config.noise, isHolonomic()); // reset noise generator
    }

    // Re-configure Critics
    obs_critic_.setConfig(config.obs_crtc);
    goal_critic_.setConfig(config.goal_crtc);
    pathfollow_critic_.setConfig(config.pathfollow_crtc);
    pathdist_critic_.setConfig(config.pathdist_crtc);
    goalangle_critic_.setConfig(config.goalangle_crtc);
    twir_critic_.setConfig(config.twir_crtc);

    std::cout << "NANO_MPPIC::MPPIc finished param update\n";
}

objects::Trajectory MPPIc::getCandidateTrajectories()
{
    return objects::Trajectory(trajectory_);
}

objects::Path MPPIc::getCurrentPlan()
{
    return objects::Path(plan_);
}

objects::Control MPPIc::getControl(const objects::Odometry2d& odom, 
                                        const objects::Path& plan)
{
    std::unique_lock<std::mutex> guard(p_lock_); // lock until param update finished

    static objects::Control output;
    
    if(not is_configured_){
        std::cout << "NANO_MPPIC::MPPIc ERROR: calling getControl() without MPPIc being configured\n";
        return output;
    }

    state_.odom = odom; // update current robot state
    plan_ = plan;

    // Reset costs
    costs_.fill(0.0);

    static bool has_failed;
    has_failed = false;

    // Predict trajectories
    do {
        predict(has_failed);
        if(has_failed)
            std::cout << "NANO_MPPIC::MPPIc prediction failed\n";
        else
            std::cout << "NANO_MPPIC::MPPIc prediction done\n";
    } while (fallback(has_failed));

    // filter control sequence (smooth out)

    // get control from sequence
    float vx = ctrl_seq_.vx(cfg_.settings.offset);
    float wz = ctrl_seq_.wz(cfg_.settings.offset);
    float vy = 0.0;
    if(isHolonomic()) vy = ctrl_seq_.vy(cfg_.settings.offset);

    output.vx = vx;
    output.vy = vy;
    output.wz = wz;

    // shift control 
    shiftControlSeq();

    return output;
}

bool MPPIc::isHolonomic()
{
    if(motion_mdl_ptr_)
        return motion_mdl_ptr_->isHolonomic();
    else 
        return false;
}

// private

void MPPIc::predict(bool &failed)
{
    for(unsigned int i=0; i < cfg_.settings.num_iters; ++i){
        generateNoisedTrajectories();   // integrate new trajectories with newly computed control inputs
        evalTrajectories(failed);       // evaluate trajectories score
        optimizeControlSeq();           // compute optimal control sequence
    }

}

bool MPPIc::fallback(bool &failed)
{
    static size_t count = 0;

    if(not failed){
        count = 0;
        return false;
    }

    reset();

    if(++count > cfg_.settings.num_retry){
        count = 0;
        throw std::runtime_error("NANO_MPPIC::MPPIc Error: failed to compute any path");
        return false;
    }

    return true;
}

void MPPIc::generateNoisedTrajectories()
{
    noise_gen_.getControls(state_, ctrl_seq_);  // generate control noise
    updateState(state_, trajectory_);           // predict trajectories with new controls 
}

void MPPIc::evalTrajectories(bool &failed)
{
    /* To-Do:
        - Define Critics Manager 
        - score() each critic
    */

    obs_critic_.score(state_, trajectory_, plan_, costs_, failed);
    goal_critic_.score(state_, trajectory_, plan_, costs_, failed);
    pathfollow_critic_.score(state_, trajectory_, plan_, costs_, failed);
    goalangle_critic_.score(state_, trajectory_, plan_, costs_, failed);
    // pathdist_critic_.score(state_, trajectory_, plan_, costs_, failed); // too much overhead
    twir_critic_.score(state_, trajectory_, plan_, costs_, failed);
}

void MPPIc::optimizeControlSeq()
{
    auto bounded_noises_vx = state_.cvx - ctrl_seq_.vx;
    auto bounded_noises_wz = state_.cwz - ctrl_seq_.wz;
    xt::noalias(costs_) += 
        cfg_.gamma / std::pow(cfg_.noise.std_vx, 2) * xt::sum(
                                                    xt::view(ctrl_seq_.vx, xt::newaxis(), xt::all()) 
                                                        * bounded_noises_vx, 1, immediate
                                                        );
    xt::noalias(costs_) +=
        cfg_.gamma / std::pow(cfg_.noise.std_wz, 2) * xt::sum(
                                                    xt::view(ctrl_seq_.wz, xt::newaxis(), xt::all()) 
                                                    * bounded_noises_wz, 1, immediate
                                                    );

    if (isHolonomic()) {
        auto bounded_noises_vy = state_.cvy - ctrl_seq_.vy;
        xt::noalias(costs_) +=
            cfg_.gamma / std::pow(cfg_.noise.std_vy, 2) * xt::sum(
                                                        xt::view(ctrl_seq_.vy, xt::newaxis(), xt::all()) 
                                                        * bounded_noises_vy, 1, immediate
                                                        );
    }

    auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
    auto && exponents = xt::eval(xt::exp(-1 / cfg_.temperature * costs_normalized));
    auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
    auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

    xt::noalias(ctrl_seq_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
    xt::noalias(ctrl_seq_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
    xt::noalias(ctrl_seq_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);

    applyControlConstraints(ctrl_seq_);
}

void MPPIc::updateState(objects::State& st,
                            objects::Trajectory& traj)
{

    // Set initial velocities
    xt::noalias(xt::view(st.vx, xt::all(), 0)) = st.odom.vx;
    xt::noalias(xt::view(st.wz, xt::all(), 0)) = st.odom.wz;

    if (isHolonomic())
        xt::noalias(xt::view(st.vy, xt::all(), 0)) = st.odom.vy;

    // Propagate new controls/velocities
    xt::noalias(xt::view(st.vx, xt::all(), xt::range(1, _))) =
        xt::view(st.cvx, xt::all(), xt::range(0, -1));

    xt::noalias(xt::view(st.wz, xt::all(), xt::range(1, _))) =
        xt::view(st.cwz, xt::all(), xt::range(0, -1));

    if (isHolonomic()) 
        xt::noalias(xt::view(st.vy, xt::all(), xt::range(1, _))) =
            xt::view(st.cvy, xt::all(), xt::range(0, -1));

    // Integrate new trajectories
    motion_mdl_ptr_->predict(st, traj);
}

void MPPIc::applyControlConstraints(objects::ControlSequence& sequence)
{
    if (isHolonomic()) {
        ctrl_seq_.vy = xt::clip(ctrl_seq_.vy, cfg_.bounds.min_vy, cfg_.bounds.max_vy);
    }

    ctrl_seq_.vx = xt::clip(ctrl_seq_.vx, cfg_.bounds.min_vx, cfg_.bounds.max_vx);
    ctrl_seq_.wz = xt::clip(ctrl_seq_.wz, cfg_.bounds.min_wz, cfg_.bounds.max_wz);

    motion_mdl_ptr_->constrainMotion(ctrl_seq_);
}

void MPPIc::shiftControlSeq()
{
    ctrl_seq_.vx = xt::roll(ctrl_seq_.vx, -1);
    ctrl_seq_.wz = xt::roll(ctrl_seq_.wz, -1);

    xt::view(ctrl_seq_.vx, -1) = xt::view(ctrl_seq_.vx, -2);
    xt::view(ctrl_seq_.wz, -1) = xt::view(ctrl_seq_.wz, -2);

    if (isHolonomic()) {
        ctrl_seq_.vy = xt::roll(ctrl_seq_.vy, -1);
        xt::view(ctrl_seq_.vy, -1) = xt::view(ctrl_seq_.vy, -2);
    }
}

} // namespace nano_mppic

   