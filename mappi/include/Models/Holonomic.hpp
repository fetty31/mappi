/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   Holonomic Motion Model. Control variables:
 *          - vx: longitudinal velocity
 *          - vy: lateral velocity
 *          - wz: yaw rate
 *   Equations:
 *      dx   = vx * cos(yaw) - vy * sin(yaw)
 *      dy   = vx * sin(yaw) + vy * cos(yaw)
 *      dyaw = w
 *
 * -----------------------------------------------------------------------------
 */

#ifndef __MAPPI_HOLONOMIC_MODEL_HPP__
#define __MAPPI_HOLONOMIC_MODEL_HPP__

#include "Models/MotionModel.hpp"

namespace mappi::models {

class Holonomic : public MotionModel {

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Holonomic object
         * 
         * @param dt 
         */
        Holonomic(double dt) : MotionModel(dt) {};

        /**
         * @brief Whether the motion model is holonomic or not
         * 
         * @return true 
         * @return false 
         */
        bool isHolonomic() override { return true; }

};

} // namespace mappi::models

#endif