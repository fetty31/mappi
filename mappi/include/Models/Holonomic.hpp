#ifndef __MAPPI_HOLONOMIC_MODEL_HPP__
#define __MAPPI_HOLONOMIC_MODEL_HPP__

#include "Models/MotionModel.hpp"

namespace mappi::models {

class Holonomic : public MotionModel {

    // FUNCTIONS

    public:

        Holonomic(float dt) : MotionModel(dt) {};

        bool isHolonomic() override { return true; }

};

} // namespace mappi::models

#endif