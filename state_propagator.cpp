#include "state_propagator.hpp"

using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace shared {

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr &si, 
                                 double &simulation_step_size,
                                 boost::shared_ptr<TorqueDamper> &damper):
    ompl::control::StatePropagator(si),
    space_information_(si),    
    model_setup_(false),
    environment_(nullptr),
    robot_(nullptr),
    simulation_step_size_(simulation_step_size),
    damper_(damper)
{
    
}

void StatePropagator::propagate(const ompl::base::State *state, 
                                const ompl::control::Control *control, 
                                const double duration, 
                                ompl::base::State *result) const {
    unsigned int dim = space_information_->getStateSpace()->getDimension() / 2;   
    
                                
    /**cout << "State: ";
    for (unsigned int i = 0; i < dim * 2.0; i++) {
        cout << " " << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }
    cout << endl;

    cout << "Torques: ";
    for (unsigned int i = 0; i < dim; i++) {
        cout << " " << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
    }
    cout << endl;*/

    
                                
    std::vector<OpenRAVE::dReal> currentJointValuesTemp;
    std::vector<OpenRAVE::dReal> currentJointVelocitiesTemp;
    std::vector<double> current_vel;    
    
    
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot_->GetJoints());    
    std::vector<OpenRAVE::dReal> input_torques;
    std::vector<OpenRAVE::dReal> damped_torques;
    for (unsigned int i = 0; i < dim; i++) {
        currentJointValuesTemp.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
        currentJointVelocitiesTemp.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i + dim]);
        input_torques.push_back(0);
        damped_torques.push_back(0);               
    }
    
    const std::vector<OpenRAVE::dReal> currentJointValues(currentJointValuesTemp);
    const std::vector<OpenRAVE::dReal> currentJointVelocities(currentJointVelocitiesTemp);
    
    
    robot_->SetDOFValues(currentJointValues);
    robot_->SetDOFVelocities(currentJointVelocities);    
    
    int num_steps = duration / simulation_step_size_;
    for (unsigned int i = 0; i < num_steps; i++) {
        robot_->GetDOFVelocities(current_vel);
        damper_->damp_torques(current_vel,
                              damped_torques);
        for (size_t k = 0; k < joints.size(); k++) {
            input_torques[k] = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[k] +
                               damped_torques[k];
            const std::vector<OpenRAVE::dReal> torques({input_torques[k]});
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

    //Enforce position limits
    for (unsigned int i = 0; i < dim; i++) {
        if (newJointValues[i] < jointsLowerPositionLimit_[i][0]) {
           newJointValues[i] = jointsLowerPositionLimit_[i][0];
        }
        else if (newJointValues[i] > jointsUpperPositionLimit_[i][0]) {
           newJointValues[i] = jointsUpperPositionLimit_[i][0];
        }

        if (newJointVelocities[i] < jointsLowerVelocityLimit_[i][0]) {
           newJointVelocities[i] = jointsLowerVelocityLimit_[i][0];
        }
        else if (newJointVelocities[i] > jointsUpperVelocityLimit_[i][0]) {
           newJointVelocities[i] = jointsUpperVelocityLimit_[i][0];
        }
        
    }
    
    for (unsigned int i = 0; i < dim; i++) {
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = newJointValues[i];
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i + dim] = newJointVelocities[i];
    }
    
    /**cout << "result ";
    for (unsigned int i = 0; i < 2 * dim; i++) {
        cout << " " << result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }
    cout << endl;
    sleep(1);*/             
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
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot_->GetJoints());
    for (size_t i = 0; i < joints.size(); i++) {
        std::vector<OpenRAVE::dReal> jointLowerLimit;
        std::vector<OpenRAVE::dReal> jointUpperLimit;
        std::vector<OpenRAVE::dReal> jointLowerVelLimit;
        std::vector<OpenRAVE::dReal> jointUpperVelLimit;
        joints[i]->GetLimits(jointLowerLimit, jointUpperLimit);
        joints[i]->GetVelocityLimits(jointLowerVelLimit, jointUpperVelLimit);
        jointsLowerPositionLimit_.push_back(jointLowerLimit);
        jointsUpperPositionLimit_.push_back(jointUpperLimit);
        jointsLowerVelocityLimit_.push_back(jointLowerVelLimit);
        jointsUpperVelocityLimit_.push_back(jointUpperVelLimit);        
    }
    model_setup_ = true;
    return model_setup_;
}

}
