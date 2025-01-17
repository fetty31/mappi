#ifndef __NANO_MPPI_ACKERMANN_MODEL_HPP__
#define __NANO_MPPI_ACKERMANN_MODEL_HPP__

#include "Models/MotionModel.hpp"
#include "Objects/Config.hpp"

namespace nano_mppic::models {

class Ackermann : public MotionModel {

    // VARIABLES

    private:    
        nano_mppic::config::AckermannModel cfg_;

    // FUNCTIONS

    public:
        explicit Ackermann(nano_mppic::config::AckermannModel& config, 
                            float dt) : MotionModel(dt) 
        {
            cfg_ = config;
        }

        void setConfig(nano_mppic::config::AckermannModel& config, float dt)
        {
            MotionModel::setConfig(dt);
            cfg_ = config;
        }

        bool isHolonomic() override { return false; }

        void constrainMotion(nano_mppic::objects::ControlSequence& ctrl_seq) override {
            auto & vx = ctrl_seq.vx;
            auto & wz = ctrl_seq.wz;
            auto wz_max = xt::sign(wz) * vx / cfg_.min_r;

            auto mask = xt::fabs(vx) / xt::fabs(wz) > cfg_.min_r;

            // Apply the result only where the mask is true
            wz = xt::where(mask, wz_max, wz);
        }
};

} // namespace nano_mppic::models

#endif