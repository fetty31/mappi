#ifndef __MAPPI_MOTIONMODEL_HPP__
#define __MAPPI_MOTIONMODEL_HPP__

#include <xtensor/xtensor.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmasked_view.hpp>

#include "Objects/ControlSequence.hpp"
#include "Objects/State.hpp"
#include "Objects/Trajectory.hpp"

#include "Utils/Auxiliar.hpp"

namespace mappi::models {

class MotionModel {

    // VARIABLES

    protected:
        float model_dt;

    // FUNCTIONS

    public:
        MotionModel(float dt) : model_dt(dt) { };

        virtual ~MotionModel() = default;

        virtual void integrate(mappi::objects::State& st, 
                            mappi::objects::Trajectory& traj)
        {
            const float initial_yaw = st.odom.yaw;

            xt::noalias(traj.yaw) =
                aux::normalize_angles(xt::cumsum(st.wz * model_dt, 1) + initial_yaw);

            const auto yaw_accum = xt::view(traj.yaw, xt::all(), xt::range(0, -1));

            auto && yaw_cos = xt::xtensor<float, 2>::from_shape(traj.yaw.shape());
            auto && yaw_sin = xt::xtensor<float, 2>::from_shape(traj.yaw.shape());
            xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = std::cos(initial_yaw);
            xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = std::sin(initial_yaw);
            xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, xt::placeholders::_))) = xt::cos(yaw_accum);
            xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, xt::placeholders::_))) = xt::sin(yaw_accum);

            auto && dx = xt::eval(st.vx * yaw_cos);
            auto && dy = xt::eval(st.vx * yaw_sin);

            if (isHolonomic()) {
                dx = dx - st.vy * yaw_sin;
                dy = dy + st.vy * yaw_cos;
            }

            xt::noalias(traj.x) = st.odom.x + xt::cumsum(dx * model_dt, 1);
            xt::noalias(traj.y) = st.odom.y + xt::cumsum(dy * model_dt, 1);
        }

        virtual void setConfig(float dt) { model_dt = dt; }

        virtual bool isHolonomic() = 0;

        virtual void constrainMotion(mappi::objects::ControlSequence&) {}
};

} // namespace mappi::models

#endif
