#ifndef UNIFORM_CONTROL_SAMPLER_TEST_HPP_
#define UNIFORM_CONTROL_SAMPLER_TEST_HPP_
#include <ompl/control/ControlSampler.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/Control.h>
#include <iostream>

namespace shared {

    class UniformControlSampler : public ompl::control::RealVectorControlUniformSampler
    {
        public:
            UniformControlSampler(const ompl::control::ControlSpace *space);
            
            void sample(ompl::control::Control *control);
            
        private:
            const ompl::control::ControlSpace *space_;
    
    };
}

#endif
