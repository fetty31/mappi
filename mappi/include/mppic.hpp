/*
 * -----------------------------------------------------------------------------
 * Author      : Oriol Martínez @fetty31
 * Created     : 2025-01-02
 * 
 * Description :
 *   MPPI controller (MPPIc) main class.
 *      - is responsible for:
 *          . Computing noise trajectories (from random controls -> integrate states)
 *          . Evaluating costs for each trajectory --> choose optimal
 *          . Returning "optimal" control sequence
 *      - owns:
 *          . critics manager (To-Do)
 *          . motion model obj
 *          . noise generator obj
 *      - has acces to:
 *          . costmap (to pass to obstacle critic, at least)
 *
 * -----------------------------------------------------------------------------
 */

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
#include "Critics/Forward.hpp"

#include "Objects/State.hpp"
#include "Objects/Trajectory.hpp"
#include "Objects/ControlSequence.hpp"
#include "Objects/Config.hpp"

#include "Utils/NoiseGenerator.hpp"
#include "Utils/CostmapInterface.hpp"
#include "Utils/Auxiliar.hpp"
#include "Utils/Filters/SavitskyGolay.hpp"
#include "Utils/Filters/LowPass.hpp"
#include "Utils/Splines/SplineAdaptor.hpp"

#include <memory>
#include <chrono>

namespace mappi {

class MPPIc {

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
        critics::Forward forward_critic_; 

        std::unique_ptr<models::MotionModel> motion_mdl_ptr_;

        utils::NoiseGenerator noise_gen_;

        xt::xtensor<double, 1> costs_;

        bool is_configured_;

        std::mutex loop_mtx;
        std::mutex reset_mtx;

    // FUNCTIONS

    public:
        /**
         * @brief Construct a new Model Predictive Path Integral controller (MPPIc) object
         * 
         */
        MPPIc();
        
        /**
         * @brief Configure MPPIc object. 
         * 
         * @param cfg Configuration struct
         * @param costmap Pointer to costmap object
         */
        void configure(config::MPPIc&,
                        shared_ptr<utils::CostmapInterface>&);
        
        /**
         * @brief Shutdown MPPIc. 
         *
         * Shuts down the noise generation object
         * 
         */
        void shutdown();
        
        /**
         * @brief Set the config object
         * 
         */
        void setConfig(config::MPPIc&);
        
        /**
         * @brief Reset MPPIc
         * 
         * Resets all time-dependant objects/variables. That is resetting
         * the current state, the computed trajectory, the last control sequence,
         * the noise generator object, the computed costs and the control sequence buffer.
         * 
         */
        void reset();

        /**
         * @brief Reset control variables
         *
         * Resets only the control variables. That is resetting 
         * the last control sequence as well as the control sequence buffer.
         * 
         */
        void resetControls();
        
        /**
         * @brief Get the output control commands
         * 
         * @param odom Current robot odometry
         * @param plan Planned trajectory
         * @return objects::Control 
         */
        objects::Control getControl(const objects::Odometry2d& odom, 
                                    const objects::Path& plan);
        
        /**
         * @brief Whether the MPPIc motion model is holonomic or not
         * 
         * @return true 
         * @return false 
         */
        bool isHolonomic();
        
        /**
         * @brief Get the candidate trajectories
         * 
         * @return objects::Trajectory 
         */
        objects::Trajectory getCandidateTrajectories();

        /**
         * @brief Get the optimal trajectory
         * 
         * @return objects::Trajectory 
         */
        objects::Trajectory getOptimalTrajectory();

        /**
         * @brief Get the current plan object
         * 
         * @return objects::Path 
         */
        objects::Path       getCurrentPlan();

        /**
         * @brief Get the current odometry
         * 
         * @return objects::Odometry2d 
         */
        objects::Odometry2d getOdometry();

    private:
        
        /**
         * @brief Predict the next trajectories.
         *
         * Performs all iterations of the MPPI algorithm calling:
         *      1. MPPIc::generateNoisedTrajectories()
         *      2. MPPIc::evalTrajectories()
         *      3. MPPIc::optimizeControlSeq()
         * 
         * @param failed Result of the MPPI algorithm (true: optimal found / false: all trajectories failed)
         */
        void predict(bool &failed);

        /**
         * @brief Performs fallback strategy when MPPIc::predict() has failed
         * 
         * @param failed Result of the last MPPI step
         * @return true 
         * @return false 
         */
        bool fallback(bool &failed);
        
        /**
         * @brief Integrate new trajectories with newly computed control inputs from the noise generator
         * 
         */
        void generateNoisedTrajectories();
        
        /**
         * @brief Compute the new trajectories score based on the active critics
         * 
         * @param failed Whether all trajectories collide with some object
         */
        void evalTrajectories(bool &failed);

        /**
         * @brief Compute optimal control sequence (Path Integral optimal equation) 
         * 
         */
        void optimizeControlSeq();

        /**
         * @brief Shifts the last computed control sequence by one 
         * 
         */
        void shiftControlSeq();
        
        /**
         * @brief Integrate new state inputs through the motion model
         * 
         * @param st Updated state with newly computed control inputs
         * @param traj Output trajectories
         */
        void updateState(objects::State&,
                        objects::Trajectory&);
        
        /**
         * @brief Apply control variables max/min boundaries and motion constraints (if any)
         * 
         */
        void applyControlConstraints();
        
        /**
         * @brief Set the plan free space
         * 
         * Checks the costmap value of each point of the current plan and fills the free variable.
         * (This is only used by PathFollow and PathDist critics)
         * 
         */
        void setPlanFreeSpace();

};

} // namespace mappi

#endif