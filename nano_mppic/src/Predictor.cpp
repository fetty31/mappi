#include "Predictor.hpp"

namespace nano_mppic {

using namespace xt::placeholders;
using xt::evaluation_strategy::immediate;

void Predictor::Predictor() : is_configured_(false)
{

}
        
void Predictor::configure(nano_mppic::config::Predictor& cfg, 
                            std::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
    cfg_ = cfg;
    
    // depending on cfg.settings.motion_model -> declare model
    // if(cfg.settings.motion_model == "Ackermann")
    motion_mdl_ptr_ = std::make_unique<nano_mppic::models::Ackermann>(cfg_.ackermann, cfg_.model_dt);
    // else
    //     motion_mdl_ptr_ = std::make_unique<nano_mppic::models::MotionModel>(cfg_.model_dt);

    noise_gen_.configure(cfg.noise, isHolonomic());

    obs_critic_.configure("obstacles_critic", costmap);

    this->reset(); // quick set up of obj dimensions

    is_configured_ = true;
}

void Predictor::shutdown()
{

}

void Predictor::reset()
{
    state_.reset(cfg_.settings.batch_size, cfg_.settings.time_steps);
    trajectory_.reset(cfg_.settings.batch_size, cfg_.settings.time_steps);
    ctrl_seq_.reset(cfg_.settings.time_steps);

    noise_gen.reset(cfg_.noise, isHolonomic());

    costs_ = xt::zeros<float>({settings_.batch_size});

}

void Predictor::getControl(const Odometry2d& odom, 
                            const Trajectory& plan){
    
    if(not is_configured_){
        std::cout << "NANO_MPPIC::Predictor ERROR: calling getControl() without Predictor being configured\n";
        return;
    }

    state_.odom = odom; // update current robot state

    // Predict trajectories
    predict();
    // filter control sequence (smooth out)
    // get control from sequence
    // shift control ?

}

bool Predictor::isHolonomic()
{
    if(motion_mdl_ptr_)
        return motion_mdl_ptr_->isHolonomic();
    else 
        return false;
}

// private

void Predictor::predict()
{
    for(unsigned int i=0; i < cfg_.settings.num_iters; ++i){
        generateNoisedTrajectories();
        evalTrajectories();
        updateControlSeq();
    }

}

void Predictor::generateNoisedTrajectories()
{
    noise_gen_.getControls(state_, ctrl_seq_);  // generate control noise
    updateState(state_, trajectory_);           // predict trajectories with new controls 
}

void Predictor::evalTrajectories()
{
    /* To-Do:
        - define Critic class
        - Obstacles critic
            . check collision with costmap2D
            . templated class (probably)
    */


    
}

void Predictor::updateControlSeq()
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

    applyControlConstraints();
}

void Predictor::updateState(nano_mppic::objects::State& st,
                        nano_mppic::objects::Trajectory& traj)
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

void Predictor::applyControlConstraints(nano_mppic::objects::ControlSequence& sequence)
{
    if (isHolonomic()) {
        ctrl_seq_.vy = xt::clip(ctrl_seq_.vy, cfg_.bounds.min_vy, cfg_.bounds.max_vy);
    }

    ctrl_seq_.vx = xt::clip(ctrl_seq_.vx, cfg_.bounds.min_vx, cfg_.bounds.max_vx);
    ctrl_seq_.wz = xt::clip(ctrl_seq_.wz, cfg_.bounds.min_wz, cfg_.bounds.max_wz);

    motion_mdl_ptr_->constrainMotion(ctrl_seq_);
}

} // namespace nano_mppic

   