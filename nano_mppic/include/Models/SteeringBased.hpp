#ifndef __NANO_MPPI_STEERINGBASED_MODEL_HPP__
#define __NANO_MPPI_STEERINGBASED_MODEL_HPP__

#include "Models/MotionModel.hpp"
#include "Objects/Config.hpp"

namespace nano_mppic::models {

class SteeringBased : public MotionModel {

    // VARIABLES

    private:    
        nano_mppic::config::SteeringBasedModel cfg_;

    // FUNCTIONS

    public:
        explicit SteeringBased(nano_mppic::config::SteeringBasedModel& config, 
                            float dt) : MotionModel(dt) 
        {
            cfg_ = config;
        }

        void setConfig(nano_mppic::config::SteeringBasedModel& config, float dt)
        {
            MotionModel::setConfig(dt);
            cfg_ = config;
        }

        bool isHolonomic() override { return false; }

        void predict(nano_mppic::objects::State& st, 
                    nano_mppic::objects::Trajectory& traj) override 
        {
            const double initial_yaw = st.odom.yaw;

            auto steer = xt::view(st.wz, xt::all(), xt::all());

            /* Kinematic Bicycle model:
            inputs: [steer, vx]
            outputs: [x, y, yaw]
            sys:
                dx = vx * cos(yaw)
                dy = vx * sin(yaw)
                dyaw = vx * tan(steer) / L
            */

            xt::noalias(traj.yaw) =
                aux::normalize_angles(xt::cumsum( st.vx * tan(steer) / cfg_.length * model_dt, 1) + initial_yaw);

            const auto yaw_accum = xt::view(traj.yaw, xt::all(), xt::range(0, -1));

            auto && yaw_cos = xt::xtensor<float, 2>::from_shape(traj.yaw.shape());
            auto && yaw_sin = xt::xtensor<float, 2>::from_shape(traj.yaw.shape());
            xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = std::cos(initial_yaw);
            xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = std::sin(initial_yaw);
            xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, xt::placeholders::_))) = xt::cos(yaw_accum);
            xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, xt::placeholders::_))) = xt::sin(yaw_accum);

            auto && dx = xt::eval(st.vx * yaw_cos);
            auto && dy = xt::eval(st.vx * yaw_sin);

            xt::noalias(traj.x) = st.odom.x + xt::cumsum(dx * model_dt, 1);
            xt::noalias(traj.y) = st.odom.y + xt::cumsum(dy * model_dt, 1);
        }
};

} // namespace nano_mppic::models

#endif