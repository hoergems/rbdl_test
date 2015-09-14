#include "state_propagator.hpp"

using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace shared {

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr &si):
    ompl::control::StatePropagator(si),
    space_information_(si),
    model_(new RBD::Model()),
    model_setup_(false) 
{
    
}

void StatePropagator::propagate(const ompl::base::State *state, 
                                const ompl::control::Control *control, 
                                const double duration, 
                                ompl::base::State *result) const {
    // The input state vector
    VectorNd q = VectorNd::Zero(model_->dof_count);
    
    // The input velocity vector 
    VectorNd qDot = VectorNd::Zero(model_->dof_count);
    
    // The input torque vector
    VectorNd tau = VectorNd::Zero (model_->dof_count);   
    for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
        q[i] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
        tau[i] = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
    }
    
    // The resulting acceleration vector
    VectorNd qDDot = VectorNd::Zero (model_->dof_count);
    
    // Calculate the forward dynamics and store the result in qDDot
    ForwardDynamics(*model_, q, qDot, tau, qDDot);
    
    //Double integration to receive the resulting state vector from qDDot
    double qDot_res = 0.0;    
    for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) { 
        qDot_res = duration * qDDot[i];        
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = duration * qDot_res;
    }                        
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

bool StatePropagator::setupModel(const char *model_file) {
    if (!Addons::URDFReadFromFile (model_file, model_, false)) {
		std::cerr << "Error loading model ./model2.urdf" << std::endl;
		abort();
		return false;
	}
	
	model_setup_ = true;
	return model_setup_;
}

}
