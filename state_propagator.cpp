#include "state_propagator.hpp"

using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace shared {

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr &si, double &simulation_step_size):
    ompl::control::StatePropagator(si),
    space_information_(si),    
    model_setup_(false),
    environment_(nullptr),
    robot_(nullptr),
    simulation_step_size_(simulation_step_size)
{
    
}

void StatePropagator::propagate(const ompl::base::State *state, 
                                const ompl::control::Control *control, 
                                const double duration, 
                                ompl::base::State *result) const {
    unsigned int dim = space_information_->getStateSpace()->getDimension() / 2;

    std::vector<OpenRAVE::dReal> upper;
    std::vector<OpenRAVE::dReal> lower;
    robot_->GetDOFLimits(lower, upper);
    
    
    cout << "lower limits position: ";
    for (unsigned int i = 0; i < dim; i++) {
        cout << lower[i] << " " << endl;
    }
    cout << endl;
    
    cout << "upper limits position: ";
    for (unsigned int i = 0; i < dim; i++) {
        cout << upper[i] << " " << endl;
    }
    cout << endl;
    
    std::vector<OpenRAVE::dReal> max_velocities;
    robot_->GetDOFVelocityLimits(max_velocities);
    cout << "velocity limit: "; 
    for (unsigned int i = 0; i < dim; i++) {
        cout << max_velocities[i] << " " << endl;
    }
    cout << endl;
    
    std::vector<OpenRAVE::dReal> acc_limits;
    robot_->GetDOFAccelerationLimits(acc_limits);

    cout << "acceleration limit: "; 
    for (unsigned int i = 0; i < dim; i++) {
        cout << acc_limits[i] << " " << endl;
    }
    cout << endl;

    std::vector<OpenRAVE::dReal> torque_limits;
    robot_->GetDOFTorqueLimits(torque_limits);

    cout << "torque limit: "; 
    for (unsigned int i = 0; i < dim; i++) {
        cout << torque_limits[i] << " " << endl;
    }
    cout << endl;
    
                                
    cout << "State: ";
    for (unsigned int i = 0; i < dim * 2.0; i++) {
        cout << " " << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }
    cout << endl;
                                
    std::vector<OpenRAVE::dReal> currentJointValuesTemp;
    std::vector<OpenRAVE::dReal> currentJointVelocitiesTemp;
    std::vector<OpenRAVE::dReal> torquesTemp;
    
    
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot_->GetJoints());
    
    for (unsigned int i = 0; i < dim; i++) {
        currentJointValuesTemp.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
        currentJointVelocitiesTemp.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i + dim]);
        torquesTemp.push_back(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i]);        
    }
    
    const std::vector<OpenRAVE::dReal> currentJointValues(currentJointValuesTemp);
    const std::vector<OpenRAVE::dReal> currentJointVelocities(currentJointVelocitiesTemp);
    
    robot_->SetDOFValues(currentJointValues);
    robot_->SetDOFVelocities(currentJointVelocities);
    
    //cout << "duration " << duration << endl;
          
    cout << "Torques: ";
    for (size_t k = 0; k < joints.size(); k++) {
       cout << torquesTemp[k] << " ";       
    }
    cout << endl;
    //cout << "start sim time " << environment_->GetSimulationTime() * 1e-6 << endl;
    int num_steps = duration / simulation_step_size_;
    for (unsigned int i = 0; i < num_steps; i++) {
        for (size_t k = 0; k < joints.size(); k++) {
            const std::vector<OpenRAVE::dReal> torques({torquesTemp[k]});
            joints[k]->AddTorque(torques);
        }        
        environment_->StepSimulation(simulation_step_size_);
        environment_->StopSimulation();
    }
    
    
    //cout << "end sim time " << environment_->GetSimulationTime() * 1e-6 << endl;
    
    std::vector<OpenRAVE::dReal> newJointValues;
    std::vector<OpenRAVE::dReal> newJointVelocities;
    
    robot_->GetDOFValues(newJointValues);
    robot_->GetDOFVelocities(newJointVelocities);
    
    for (unsigned int i = 0; i < dim; i++) {
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = newJointValues[i];
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i + dim] = newJointVelocities[i];
    }
    
    cout << "result ";
    for (unsigned int i = 0; i < 2 * dim; i++) {
        cout << " " << result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }
    cout << endl;
    cout << endl;
    //sleep(1);             
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

bool StatePropagator::setupOpenRAVEEnvironment(OpenRAVE::EnvironmentBasePtr environment,
                                               OpenRAVE::RobotBasePtr robot) {
    environment_ = environment;
    robot_ = robot;
    model_setup_ = true;
    return model_setup_;
}

}
