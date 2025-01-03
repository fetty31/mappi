#ifndef __NANO_MPPIC_PREDICTOR_HPP__
#define __NANO_MPPIC_PREDICTOR_HPP__

namespace nano_mppic {

class Predictor {

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

}

#endif