/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Rear CoG Bicycle Kinematic Motion Model. Control variables:
 *          - vx: longitudinal velocity
 *          - wz: steering rate (steering velocity)
 *   Equations:
 *      dx   = vx * cos(yaw)
 *      dy   = vx * sin(yaw)
 *      dyaw = vx * tan(steering)/length
 *      dsteering = wz
 * where: 
 *      - length == robot length
 *      - steering == robot steering angle (at wheel reference)
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_BICYCLEKINREAR_MODEL_HPP__
#define __MAPPI_BICYCLEKINREAR_MODEL_HPP__

#include "Models/MotionModel.hpp"
#include "Objects/Config.hpp"

namespace mappi::models {

class BicycleKinRear : public MotionModel {

    // VARIABLES

    private:    
        mappi::config::BicycleKinModel cfg_;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Bicycle Kin object
         * 
         * @param config 
         * @param dt 
         */
        explicit BicycleKinRear(mappi::config::BicycleKinModel& config, 
                                    double dt) : MotionModel(dt) 
        {
            cfg_ = config;
        }

        /**
         * @brief Configure the Bicycle Kin model
         * 
         * @param config Configuration object
         * @param dt Sampling time [s]
         */
        void setConfig(mappi::config::BicycleKinModel& config, double dt)
        {
            MotionModel::setConfig(dt);
            cfg_ = config;
        }

        /**
         * @brief Whether the motion model is holonomic or not
         * 
         * @return true 
         * @return false 
         */
        bool isHolonomic() override { return false; }

        /**
         * @brief Integrate (predict) all sampled trajectories knowing the new computed states
         * 
         * @param st Current state
         * @param traj Output trajectories 
         */
        void integrate(mappi::objects::State& st, 
                    mappi::objects::Trajectory& traj) override 
        {
            const double initial_yaw = st.odom.yaw;
            const double initial_steering = st.odom.steering;

            auto && steer = xt::xtensor<double, 2>::from_shape(st.wz.shape()); // abuse of notation, here wz == steering rate
            xt::noalias(steer) = (xt::cumsum(st.wz * model_dt, 1) + initial_steering);

            // Limit steering angle
            auto steer_limit = xt::sign(steer) * cfg_.max_steer;
            auto mask = xt::fabs(steer) > cfg_.max_steer;
            steer = xt::where(mask, steer_limit, steer);

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

            auto && yaw_cos = xt::xtensor<double, 2>::from_shape(traj.yaw.shape());
            auto && yaw_sin = xt::xtensor<double, 2>::from_shape(traj.yaw.shape());
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

} // namespace mappi::models

#endif