#include "control_sampler.hpp"

namespace shared {

UniformControlSampler::UniformControlSampler(const ompl::control::ControlSpace *space):
    ompl::control::RealVectorControlUniformSampler(space),
    space_(space)
{

}

void UniformControlSampler::sample(ompl::control::Control *control) {    
    std::vector<double> low = space_->as<ompl::control::RealVectorControlSpace>()->getBounds().low;
    std::vector<double> high = space_->as<ompl::control::RealVectorControlSpace>()->getBounds().high;
    for (size_t i = 0; i < space_->getDimension(); i++) {
        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = rng_.uniformReal(low[i], high[i]); 
    }
}

}
