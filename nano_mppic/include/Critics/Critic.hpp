#ifndef __NANO_MPPI_CRITIC_HPP__
#define __NANO_MPPI_CRITIC_HPP__

namespace nano_mppic::critics {

class Critic {

    // FUNCTIONS

    public:
        Critic() = default;

        virtual ~Critic() = default;

        virtual void score();

};

} // namespace nano_mppic::critics

#endif