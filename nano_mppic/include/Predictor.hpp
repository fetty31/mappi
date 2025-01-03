#ifndef __NANO_MPPIC_PREDICTOR_HPP__
#define __NANO_MPPIC_PREDICTOR_HPP__

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

    public:

    private:

    // FUNCTIONS

    public:
        Predictor();
        
        void configure(/*Config cfg*/);

        void shutdown();

        void reset();

        void getControl();

    private:

        void predict();

        void generateNoisedTrajectories();
        void evalTrajectories();
        void updateControl();

};

} // namespace nano_mppic

#endif