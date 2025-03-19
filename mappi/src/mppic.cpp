#include "mppic.hpp"

namespace mappi {

using namespace xt::placeholders;
using xt::evaluation_strategy::immediate;

MPPIc::MPPIc() : is_configured_(false) { }
        
void MPPIc::configure(config::MPPIc& cfg, 
                            shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
    cfg_ = cfg;
    
    if(cfg.settings.motion_model == "Ackermann")
        motion_mdl_ptr_ = std::make_unique<models::Ackermann>(cfg.ackermann, cfg.model_dt);
    else if(cfg.settings.motion_model == "BicycleKin")
        motion_mdl_ptr_ = std::make_unique<models::BicycleKin>(cfg.steering, cfg.model_dt);
    else
        motion_mdl_ptr_ = std::make_unique<models::Holonomic>(cfg.model_dt);

    noise_gen_.configure(cfg.noise, isHolonomic());

    obs_critic_.configure("obstacles_critic", cfg.obs_crtc, costmap);
    goal_critic_.configure("goal_critic", cfg.goal_crtc, costmap);
    pathfollow_critic_.configure("pathfollow_critic", cfg.pathfollow_crtc, costmap);
    pathangle_critic_.configure("pathangle_critic", cfg.pathangle_crtc, costmap);
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
    reset_mtx.lock(); // avoid accessing objects while being reset

    state_.reset(cfg_.noise.batch_size, cfg_.noise.time_steps);
    trajectory_.reset(cfg_.noise.batch_size, cfg_.noise.time_steps);
    ctrl_seq_.reset(cfg_.noise.time_steps);
    
    noise_gen_.reset(cfg_.noise, isHolonomic());

    costs_ = xt::zeros<float>({cfg_.noise.batch_size});

    ctrl_history_[0] = {0.0, 0.0, 0.0};
    ctrl_history_[1] = {0.0, 0.0, 0.0};
    ctrl_history_[2] = {0.0, 0.0, 0.0};
    ctrl_history_[3] = {0.0, 0.0, 0.0};

    reset_mtx.unlock(); 
}

void MPPIc::resetControls()
{
    reset_mtx.lock();
    ctrl_seq_.reset(cfg_.noise.time_steps);
    ctrl_history_[0] = {0.0, 0.0, 0.0};
    ctrl_history_[1] = {0.0, 0.0, 0.0};
    ctrl_history_[2] = {0.0, 0.0, 0.0};
    ctrl_history_[3] = {0.0, 0.0, 0.0};
    reset_mtx.unlock();
}

void MPPIc::setConfig(config::MPPIc& cfg)
{
    loop_mtx.lock(); // lock control loop

    std::cout << "mappi::MPPIc updating parameters...\n";

    if(cfg.settings.motion_model == "Ackermann")
        motion_mdl_ptr_ = std::make_unique<models::Ackermann>(cfg.ackermann, cfg.model_dt);
    else if(cfg.settings.motion_model == "BicycleKin")
        motion_mdl_ptr_ = std::make_unique<models::BicycleKin>(cfg.steering, cfg.model_dt);
    else
        motion_mdl_ptr_ = std::make_unique<models::Holonomic>(cfg.model_dt);

    std::cout << "mappi::MPPIc finished with MotionModel update\n";

    if( (cfg.noise.batch_size != cfg_.noise.batch_size) || 
        (cfg.noise.time_steps != cfg_.noise.time_steps) )
    {

        std::cout << "mappi::MPPIc resetting all mppi controller\n";

        cfg_ = cfg;
        reset(); // reset full MPPIc
    }
    else
    {
        std::cout << "mappi::MPPIc resetting only noise generator\n";

        cfg_ = cfg;
        noise_gen_.reset(cfg.noise, isHolonomic()); // reset noise generator
    }

    std::cout << "mappi::MPPIc updating Critics...\n";

    // Re-configure Critics
    obs_critic_.setConfig(cfg.obs_crtc);
    goal_critic_.setConfig(cfg.goal_crtc);
    pathfollow_critic_.setConfig(cfg.pathfollow_crtc);
    pathangle_critic_.setConfig(cfg.pathangle_crtc);
    pathdist_critic_.setConfig(cfg.pathdist_crtc);
    goalangle_critic_.setConfig(cfg.goalangle_crtc);
    twir_critic_.setConfig(cfg.twir_crtc);

    loop_mtx.unlock(); // unlock control loop

    std::cout << "mappi::MPPIc finished param update\n";
}

objects::Trajectory MPPIc::getCandidateTrajectories()
{
    reset_mtx.lock(); // avoid access while being reset
        objects::Trajectory traj_cpy(trajectory_);
    reset_mtx.unlock();

    return std::move(traj_cpy);
}

objects::Path MPPIc::getCurrentPlan()
{
    reset_mtx.lock(); // avoid access while being reset
        objects::Path plan_cpy(plan_);
    reset_mtx.unlock();

    return std::move(plan_cpy);
}

objects::Trajectory MPPIc::getOptimalTrajectory()
{
    reset_mtx.lock(); // avoid access while being reset
    
    objects::Trajectory opt_trajectory; 
    opt_trajectory.reset(1, trajectory_.x.shape(1)); // the one and only optimal trajectory

    objects::State opt_state;
    opt_state.reset(1, ctrl_seq_.vx.size());

    opt_state.odom = state_.odom; // last received odom
    xt::view(opt_state.vx, xt::all(), xt::all()) = ctrl_seq_.vx;
    xt::view(opt_state.vy, xt::all(), xt::all()) = ctrl_seq_.vy;
    xt::view(opt_state.wz, xt::all(), xt::all()) = ctrl_seq_.wz;

    motion_mdl_ptr_->integrate(opt_state, opt_trajectory);

    reset_mtx.unlock(); 

    return std::move(opt_trajectory);
}

objects::Control MPPIc::getControl(const objects::Odometry2d& odom, 
                                        const objects::Path& plan)
{
    loop_mtx.lock(); // lock main control loop

    static objects::Control output;

    if(not is_configured_){
        std::cout << "mappi::MPPIc ERROR: calling getControl() without MPPIc being configured\n";
        return objects::Control();
    }

    // Update current robot state
    state_.odom = odom; 

    // (optional) Interpolate received plan 
    if(cfg_.settings.use_splines){
        spline::BSpline spline(plan);
        std::vector<float> u = aux::linspace<float>(0.0f, 0.99f, 100);

        objects::Path interp_plan = spline.interpolate(u, 0);
        plan_ = interp_plan;
    }else
        plan_ = plan;

    // Compute free space in received plan
    setPlanFreeSpace();

    // Reset costs
    costs_.fill(0.0);

    static bool has_failed;
    has_failed = false;

    // Predict trajectories
    do {
        predict(has_failed);

    } while (fallback(has_failed));

    // Filter control sequence (smooth out)
    filters::savitskyGolayFilter(ctrl_seq_, ctrl_history_);

    // Get control from sequence
    float vx = ctrl_seq_.vx(cfg_.settings.offset);
    float wz = ctrl_seq_.wz(cfg_.settings.offset);
    float vy = 0.0;
    if(isHolonomic()) vy = ctrl_seq_.vy(cfg_.settings.offset);

    // (optional) Low pass filter 
    // wz = filters::lowPassFilter<float>(wz);

    filters::lowPassFilter<float>(vx, vy, wz, 
                                output.vx, 
                                output.vy, 
                                output.wz );

    // Shift control 
    shiftControlSeq();

    loop_mtx.unlock();

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
        std::cout << "mappi::MPPIc prediction done\n";
        count = 0;
        return false;
    }

    std::cout << "mappi::MPPIc prediction failed\n";

    reset();

    if(++count > cfg_.settings.num_retry){
        count = 0;
        throw std::runtime_error("mappi::MPPIc ERROR: failed to compute any path");
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
    pathangle_critic_.score(state_, trajectory_, plan_, costs_, failed);
    goalangle_critic_.score(state_, trajectory_, plan_, costs_, failed);
    pathdist_critic_.score(state_, trajectory_, plan_, costs_, failed); // too much overhead
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
    motion_mdl_ptr_->integrate(st, traj);
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

void MPPIc::setPlanFreeSpace()
{
    for(size_t i=0; i < plan_.x.size(); ++i){
        unsigned char cost_c = pathfollow_critic_.costAtPose(plan_.x(i), plan_.y(i));

        if(pathfollow_critic_.isInCollision(cost_c)){
            plan_.free(i) = false;
        }else{
            plan_.free(i) = true;
        }
    }
}

} // namespace mappi

   