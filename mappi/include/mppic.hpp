#ifndef __MAPPI_HPP__
#define __MAPPI_HPP__

#include "Models/Ackermann.hpp"
#include "Models/Holonomic.hpp"
#include "Models/BicycleKin.hpp"

#include "Critics/Obstacles.hpp"
#include "Critics/Goal.hpp"
#include "Critics/Twirling.hpp"
#include "Critics/PathFollow.hpp"
#include "Critics/PathAngle.hpp"
#include "Critics/PathDist.hpp"
#include "Critics/GoalAngle.hpp"

#include "Objects/State.hpp"
#include "Objects/Trajectory.hpp"
#include "Objects/ControlSequence.hpp"
#include "Objects/Config.hpp"

#include "Utils/NoiseGenerator.hpp"
#include "Utils/Auxiliar.hpp"
#include "Utils/SplineAdaptor.hpp"

#include <memory>
#include <chrono>

namespace mappi {

class MPPIc {

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
        config::MPPIc cfg_;

        objects::ControlSequence ctrl_seq_;
        objects::State state_;
        objects::Trajectory trajectory_;
        objects::Path plan_;

        std::array<objects::Control,4> ctrl_history_;

        // To-Do: define critics manager 
        critics::Obstacles obs_critic_; 
        critics::Goal goal_critic_; 
        critics::PathFollow pathfollow_critic_; 
        critics::PathAngle pathangle_critic_; 
        critics::GoalAngle goalangle_critic_; 
        critics::PathDist pathdist_critic_; 
        critics::Twirling twir_critic_; 

        std::unique_ptr<models::MotionModel> motion_mdl_ptr_;

        utils::NoiseGenerator noise_gen_;

        xt::xtensor<float, 1> costs_;

        bool is_configured_;

        std::mutex loop_mtx;
        std::mutex reset_mtx;

    // FUNCTIONS

    public:
        MPPIc();
        
        void configure(config::MPPIc&,
                        mappi::shared_ptr<costmap_2d::Costmap2DROS>&);

        void shutdown();

        void setConfig(config::MPPIc&);

        void reset();
        void resetControls();

        objects::Control getControl(const objects::Odometry2d& odom, 
                                    const objects::Path& plan);
        
        bool isHolonomic();

        objects::Trajectory getCandidateTrajectories();
        objects::Trajectory getOptimalTrajectory();
        objects::Path       getCurrentPlan();

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

        void setPlanFreeSpace();

};

} // namespace mappi

#endif