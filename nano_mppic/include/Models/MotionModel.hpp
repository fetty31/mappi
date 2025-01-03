#ifndef __NANO_MPPI_MOTIONMODEL_HPP__
#define __NANO_MPPI_MOTIONMODEL_HPP__

namespace nano_mppic::models {

class MotionModel {

    public:
        MotionModel() = default;

        virtual ~MotionModel() = default;

        virtual void predict(nano_mppic::objects::State & state)
        {
            using namespace xt::placeholders;  // NOLINT
            xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
                xt::view(state.cvx, xt::all(), xt::range(0, -1));

            xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
                xt::view(state.cwz, xt::all(), xt::range(0, -1));

            if (isHolonomic()) 
                xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
                    xt::view(state.cvy, xt::all(), xt::range(0, -1));
        }

        virtual bool isHolonomic() = 0;

        virtual void constrainMotion(nano_mppic::objects::ControlSequence ctrl_seq) {}
};

} // namespace nano_mppic::models

#endif
