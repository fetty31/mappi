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
        Holonomic(float dt) : MotionModel(dt) {};

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