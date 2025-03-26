#ifndef __MAPPI_BICYCLEKIN_MODEL_HPP__
#define __MAPPI_BICYCLEKIN_MODEL_HPP__

#include "Models/MotionModel.hpp"
#include "Objects/Config.hpp"

namespace mappi::models {

class BicycleKin : public MotionModel {

    // VARIABLES

    private:    
        mappi::config::BicycleKinModel cfg_;

    // FUNCTIONS

    public:
        explicit BicycleKin(mappi::config::BicycleKinModel& config, 
                            float dt) : MotionModel(dt) 
        {
            cfg_ = config;
        }

        void setConfig(mappi::config::BicycleKinModel& config, float dt)
        {
            MotionModel::setConfig(dt);
            cfg_ = config;
        }

        bool isHolonomic() override { return false; }

        void integrate(mappi::objects::State& st, 
                    mappi::objects::Trajectory& traj) override 
        {
            const float initial_yaw = st.odom.yaw;
            const float initial_steering = st.odom.steering;

            auto && steer = xt::xtensor<float, 2>::from_shape(st.wz.shape()); // abuse of notation, here wz == steering rate
            xt::noalias(steer) = (xt::cumsum(st.wz * model_dt, 1) + initial_steering);

            // auto steer = xt::view(st.wz, xt::all(), xt::all()); 

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

        void constrainMotion(mappi::objects::ControlSequence& ctrl_seq) override {
            auto & wz = ctrl_seq.wz;
            float steer_max = 0.26;

            auto wz_max = xt::sign(wz) * steer_max;

            auto mask = xt::fabs(wz) > steer_max;

            // Apply the result only where the mask is true
            wz = xt::where(mask, wz_max, wz);
        }
};

} // namespace mappi::models

#endif