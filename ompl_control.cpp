#include "ompl_control.hpp"

using std::cout;
using std::endl;

namespace shared {

OMPLControlTest::OMPLControlTest() {

}

bool OMPLControlTest::isValid(const ompl::base::State *state) {
    return true;
}

void OMPLControlTest::test() {
    // Dimension of the state space 
    int space_dimension = 3;
    
    // Space bounds    
    ompl::base::RealVectorBounds bounds(space_dimension);
    bounds.setLow(-2.0 * M_PI);
    bounds.setHigh(2.0 * M_PI);  

    // The state space  
    ompl::base::StateSpacePtr space_(new ompl::base::RealVectorStateSpace(space_dimension));
    space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    
    // The control space
    ompl::control::ControlSpacePtr control_space_(new ompl::control::RealVectorControlSpace(space_, space_dimension));
    
    // The space information
    ompl::control::SpaceInformationPtr space_information_(new ompl::control::SpaceInformation(space_, control_space_));
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    
    // The state propagator
    ompl::control::StatePropagatorPtr state_propagator_(new StatePropagator(space_information_));
    space_information_->setStatePropagator(state_propagator_);
    
    // The problem definition
    ompl::base::ProblemDefinitionPtr problem_definition_(new ompl::base::ProblemDefinition(space_information_));
    
    // The planner (unidirectional RRT)
    ompl::base::PlannerPtr planner_(new ompl::control::RRT(space_information_));    
    planner_->as<ompl::control::RRT>()->setIntermediateStates(false);
    
    //problem_definition_->addStartState(start_state);
    //problem_definition_->setGoal(goal_);
    
    cout << "hello" << endl;
}
 
}

int main(int argc, char** argv) {
    shared::OMPLControlTest ompl_test;
    ompl_test.test();
    return 0;
}


