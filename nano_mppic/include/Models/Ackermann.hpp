#ifndef __NANO_MPPI_ACKERMANN_MODEL_HPP__
#define __NANO_MPPI_ACKERMANN_MODEL_HPP__

#include "Models/MotionModel.hpp"
#include "Objects/Config.hpp"

namespace nano_mppic::models {

class Ackermann : public MotionModel {

    // VARIABLES

    private:    
        float min_radius_;

    // FUNCTIONS

    public:
        explicit Ackermann(nano_mppic::config::AckermannModel& cfg, 
                            float dt) : MotionModel(dt) 
        {
            min_radius_ = cfg.min_r;
        }

        bool isHolonomic() override { return false; }

        void constrainMotion(nano_mppic::objects::ControlSequence& ctrl_seq) override {
            auto & vx = ctrl_seq.vx;
            auto & wz = ctrl_seq.wz;

            auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) > min_radius_);
            view = xt::sign(wz) * vx / min_radius_;
        }

        float getMinTurningRadius() { return min_radius_; }

};

} // namespace nano_mppic::models

#endif