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
    
    /** Setup the control space properly here */
    
    // The space information
    ompl::control::SpaceInformationPtr space_information_(new ompl::control::SpaceInformation(space_, control_space_));
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(0.03);
    
    // The state propagator
    ompl::control::StatePropagatorPtr state_propagator_(new StatePropagator(space_information_));    
    boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupModel("./model2.urdf");    
    space_information_->setStatePropagator(state_propagator_);
    
    // The problem definition
    ompl::base::ProblemDefinitionPtr problem_definition_(new ompl::base::ProblemDefinition(space_information_));
    
    // The planner (unidirectional RRT)
    ompl::base::PlannerPtr planner_(new ompl::control::RRT(space_information_));    
    planner_->as<ompl::control::RRT>()->setIntermediateStates(false);
    planner_->setProblemDefinition(problem_definition_);    
    
    
    ompl::base::ScopedState<> start_state(space_);
    start_state[0] = 0.0;
    start_state[1] = 0.0;
    start_state[2] = 0.0;
    
    ompl::base::ScopedState<> goal_state(space_);
    goal_state[0] = -1.57;
    goal_state[1] = 0.0;
    goal_state[2] = 0.0; 
    
    problem_definition_->addStartState(start_state);
    problem_definition_->setGoalState(goal_state);
    
    bool solved = false;
    
    while (!solved) {
        solved = planner_->solve(10);
    }
    cout << "solved " << solved << endl;
    
    if (solved) {
        boost::shared_ptr<ompl::control::PathControl> solution_path_ = 
            boost::static_pointer_cast<ompl::control::PathControl>(problem_definition_->getSolutionPath());
        cout << "Length of solution path " << solution_path_->length() << endl;
        std::vector<ompl::base::State*> solution_states_(solution_path_->getStates());
        for (size_t i = 0; i < solution_states_.size(); i++) {
            cout << "State: ";
            for (size_t j = 0; j < 3; j++) {
                cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << ", ";
            }
            cout << endl;
        }
    }
}
 
}

int main(int argc, char** argv) {
    shared::OMPLControlTest ompl_test;
    ompl_test.test();
    return 0;
}


