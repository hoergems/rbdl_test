#include "ompl_control.hpp"

using std::cout;
using std::endl;

namespace shared {

OMPLControlTest::OMPLControlTest(unsigned int &state_space_dimension,
                                 unsigned int &control_space_dimension):
    state_space_dimension_(state_space_dimension),
    control_space_dimension_(control_space_dimension),
    state_space_(new ompl::base::RealVectorStateSpace(state_space_dimension_)),
    control_space_(new ompl::control::RealVectorControlSpace(state_space_, 
                                                             control_space_dimension_)),
    space_information_(new ompl::control::SpaceInformation(state_space_, control_space_)),
    problem_definition_(new ompl::base::ProblemDefinition(space_information_)),
    planner_(new ompl::control::RRT(space_information_)) {

}

bool OMPLControlTest::isValid(const ompl::base::State *state) {
    return state_space_->as<ompl::base::RealVectorStateSpace>()->satisfiesBounds(state);
}

bool OMPLControlTest::solve_() {
    bool solved = false;
    bool hasExactSolution = false;
    while (!solved && !hasExactSolution) {
        solved = planner_->solve(10.0);
        
        // Get all the solutions
        std::vector<ompl::base::PlannerSolution> solutions = problem_definition_->getSolutions();
        for (size_t i = 0; i < solutions.size(); i++) {
            if (!solutions[i].approximate_) {
                hasExactSolution = true;
                break;
            }
        }
        // Check if there's an exact solution
    }

    return true;
}

void OMPLControlTest::test() {
    // Space bounds    
    ompl::base::RealVectorBounds state_bounds(state_space_dimension_);
    state_bounds.setLow(-2.0 * M_PI);
    state_bounds.setHigh(2.0 * M_PI);    
    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_bounds);

    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);
    control_bounds.setLow(-3.0);
    control_bounds.setHigh(3.0);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);    
    
    
    /** Setup the control space properly here */
    
    // The space information    
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    space_information_->setMinMaxControlDuration(1, 2);
    space_information_->setPropagationStepSize(0.03);
    
    // The state propagator
    ompl::control::StatePropagatorPtr state_propagator_(new StatePropagator(space_information_));    
    boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupModel("./model2.urdf");    
    space_information_->setStatePropagator(state_propagator_);
        
    planner_->as<ompl::control::RRT>()->setIntermediateStates(false);
    planner_->setProblemDefinition(problem_definition_);    
    
    // Set the start and goal state
    ompl::base::ScopedState<> start_state(state_space_);
    start_state[0] = 0.0;
    start_state[1] = 0.0;
    start_state[2] = 0.0;
    start_state[3] = 0.0;
    start_state[4] = 0.0;
    start_state[5] = 0.0;
    
    ompl::base::ScopedState<> goal_state(state_space_);
    goal_state[0] = -1.57;
    goal_state[1] = 1.57;
    goal_state[2] = 0.0;
    goal_state[3] = 0.0;
    goal_state[4] = 0.0;
    goal_state[5] = 0.0;    

    const ompl::base::GoalPtr goal_ptr(new ManipulatorGoalState(space_information_, goal_state)); 
    
    problem_definition_->addStartState(start_state);
    //problem_definition_->setGoalState(goal_state);
    problem_definition_->setGoal(goal_ptr);
    
    bool solved = false;
    
    solved = solve_();
    cout << "solved " << endl;
    cout << "Number of solutions found: " << problem_definition_->getSolutionCount() << endl;    
    
    if (solved) {
        ompl::base::PlannerSolution planner_solution(problem_definition_->getSolutionPath());
        cout << "Solution is approximate: ";
        if (planner_solution.approximate_) {
            cout << " True";
        }
        else {
            cout << " False";
        }
        cout << endl;        
        boost::shared_ptr<ompl::control::PathControl> solution_path_ = 
            boost::static_pointer_cast<ompl::control::PathControl>(planner_solution.path_);        
        cout << "Length of solution path " << solution_path_->length() << endl;
        std::vector<ompl::base::State*> solution_states_(solution_path_->getStates());
        for (size_t i = 0; i < solution_states_.size(); i++) {
            cout << "State: ";
            for (size_t j = 0; j < 3; j++) {
                cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] << ", ";
            }
            cout << endl;
        }
    }
}
 
}

int main(int argc, char** argv) {
    unsigned int state_space_dimension = 6;
    unsigned int control_space_dimension = 3;
    shared::OMPLControlTest ompl_test(state_space_dimension, control_space_dimension);
    ompl_test.test();
    return 0;
}


