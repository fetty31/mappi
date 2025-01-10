#ifndef __NANO_MPPIC_PREDICTOR_HPP__
#define __NANO_MPPIC_PREDICTOR_HPP__

#include "Models/Ackermann.hpp"
#include "Models/Holonomic.hpp"

#include "Critics/Obstacles.hpp"

#include "Objects/State.hpp"
#include "Objects/Trajectory.hpp"
#include "Objects/ControlSequence.hpp"
#include "Objects/Config.hpp"

#include "Utils/NoiseGenerator.hpp"
#include "Utils/Auxiliar.hpp"

#include <memory>

namespace nano_mppic {

class Predictor {

    /* Description:
        - should be responsible for:
            . Compute noise trajectories (from random controls -> integrate states)
            . Evaluate costs for each trajectory --> choose optimal
            . Return "optimal" control sequence
        - should own:
            . critics manager
            . physical model obj
        - should have acces to:
            . costmap (to pass to obstacle critic, at least)
    */

    // VARIABLES

    private:
        config::Predictor cfg_;

        objects::ControlSequence ctrl_seq_;
        objects::State state_;
        objects::Trajectory trajectory_;
        objects::Path plan_;

        critics::Obstacles obs_critic_; // To-Do: define critics manager 

        std::unique_ptr<models::MotionModel> motion_mdl_ptr_;

        utils::NoiseGenerator noise_gen_;

        xt::xtensor<float, 1> costs_;

        bool is_configured_;

    // FUNCTIONS

    public:
        Predictor();
        
        void configure(config::Predictor&,
                        nano_mppic::shared_ptr<costmap_2d::Costmap2DROS>&);

        void shutdown();

        void reset();

        objects::Control getControl(const objects::Odometry2d& odom, 
                                    const objects::Path& plan);
        
        bool isHolonomic();

    private:

        void predict(bool &failed);

        bool fallback(bool &failed);

        void generateNoisedTrajectories();

        void evalTrajectories(bool &failed);

        void optimizeControlSeq();

        void shiftControlSeq();

        void updateState(objects::State&,
                        objects::Trajectory&);

        void applyControlConstraints(objects::ControlSequence&);

};

} // namespace nano_mppic

#endif