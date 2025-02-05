#ifndef __NANO_MPPIC_GUIDANCE_WRAPPER_HPP__
#define __NANO_MPPIC_GUIDANCE_WRAPPER_HPP__

#include <guidance_planner/global_guidance.h>

class GuidanceWrapper : public GuidancePlanner::GlobalGuidance {

    public:
        GuidanceWrapper() = default;
        ~GuidanceWrapper() = default;

};

#endif