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
        nano_mppic::config::Predictor cfg_;

        nano_mppic::objects::ControlSequence ctrl_seq_;
        nano_mppic::objects::State state_;
        nano_mppic::objects::Trajectory trajectory_;

        nano_mppic::critics::Obstacles obs_critic_; // To-Do: define critics manager 

        std::unique_ptr<nano_mppic::models::MotionModel> motion_mdl_ptr_;

        nano_mppic::utils::NoiseGenerator noise_gen_;

        xt::xtensor<float, 1> costs_;

        bool is_configured_;

    // FUNCTIONS

    public:
        Predictor();
        
        void configure(nano_mppic::config::Predictor&,
                        std::shared_ptr<costmap_2d::Costmap2DROS>&);

        void shutdown();

        void reset();

        void getControl(const Odometry2d& odom, 
                        const Trajectory& plan);
        
        bool isHolonomic();

    private:

        void predict();

        void generateNoisedTrajectories();

        void evalTrajectories();

        void updateControlSeq();

        void updateState(nano_mppic::objects::State&,
                        nano_mppic::objects::Trajectory&);

        void applyControlConstraints(nano_mppic::objects::ControlSequence&);

};

} // namespace nano_mppic

#endif