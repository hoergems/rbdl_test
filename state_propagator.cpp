#include "state_propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr &si):
    ompl::control::StatePropagator(si) 
{
    
}

void StatePropagator::propagate(const ompl::base::State *state, 
                                const ompl::control::Control *control, 
                                const double duration, 
                                ompl::base::State *result) const {
    cout << "Nope" << endl;                                
}

bool StatePropagator::canPropagateBackward() const{
    return false;
}

bool StatePropagator::steer(const ompl::base::State* /*from*/, 
                            const ompl::base::State* /*to*/, 
                            ompl::control::Control* /*result*/, 
                            double& /*duration*/) const {
    return false;                            
} 

bool StatePropagator::canSteer() const {
    return false;
}

}
