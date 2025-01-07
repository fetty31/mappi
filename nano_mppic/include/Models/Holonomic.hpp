#ifndef __NANO_MPPI_HOLONOMIC_MODEL_HPP__
#define __NANO_MPPI_HOLONOMIC_MODEL_HPP__

#include "Models/MotionModel.hpp"

namespace nano_mppic::models {

class Holonomic : public MotionModel {

    // FUNCTIONS

    public:

        Holonomic(float dt) : MotionModel(dt) {};

        bool isHolonomic() override { return true; }

};

} // namespace nano_mppic::models

#endif