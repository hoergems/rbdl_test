#include "ompl_control.hpp"

using std::cout;
using std::endl;

namespace shared {

OMPLControlTest::OMPLControlTest(const char *model_file,
                                 double &control_duration,
                                 unsigned int &state_space_dimension,
                                 unsigned int &control_space_dimension):
    model_file_(model_file),
    control_duration_(control_duration),
    state_space_dimension_(state_space_dimension),
    control_space_dimension_(control_space_dimension),
    state_space_(new ompl::base::RealVectorStateSpace(state_space_dimension_)),
    state_space_bounds_(state_space_dimension_),
    control_space_(new ompl::control::RealVectorControlSpace(state_space_, 
                                                             control_space_dimension_)),
    space_information_(new ompl::control::SpaceInformation(state_space_, control_space_)),
    problem_definition_(new ompl::base::ProblemDefinition(space_information_)),
    planner_(new ompl::control::RRT(space_information_)) 
{
    setup_bounds_();
}

bool OMPLControlTest::setup_bounds_() {
    // Set the lower bounds
    double vel_bound = M_PI;
    double torque_bounds = 0.2;
    state_space_bounds_.setLow(0, -2.0 * M_PI);
    state_space_bounds_.setLow(1, -2.0 * M_PI);
    state_space_bounds_.setLow(2, -2.0 * M_PI);
    state_space_bounds_.setLow(3, -vel_bound);
    state_space_bounds_.setLow(4, -vel_bound);
    state_space_bounds_.setLow(5, -vel_bound);

    // Set the higher bounds
    state_space_bounds_.setHigh(0, 2.0 * M_PI);
    state_space_bounds_.setHigh(1, 2.0 * M_PI);
    state_space_bounds_.setHigh(2, 2.0 * M_PI);
    state_space_bounds_.setHigh(3, vel_bound);
    state_space_bounds_.setHigh(4, vel_bound);
    state_space_bounds_.setHigh(5, vel_bound);

    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_space_bounds_);

    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);
    control_bounds.setLow(-torque_bounds);
    control_bounds.setHigh(torque_bounds);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds); 
    return true;
}

bool OMPLControlTest::isValid(const ompl::base::State *state) {
    return state_space_->as<ompl::base::RealVectorStateSpace>()->satisfiesBounds(state);
}

bool OMPLControlTest::solve_() {
    bool solved = false;
    bool hasExactSolution = false;
    boost::timer t;
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
    cout << "Solution found in " << t.elapsed() << "seconds" << endl;
    return true;
}

void OMPLControlTest::test() {
    
    /** Setup the control space properly here */
    
    // The space information    
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(control_duration_);
    
    // The state propagator
    ompl::control::StatePropagatorPtr state_propagator_(new StatePropagator(space_information_));    
    boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupModel(model_file_);    
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
    goal_state[0] = 0.5;
    goal_state[1] = 0.0;
    goal_state[2] = 0.0;
    goal_state[3] = 0.0;
    goal_state[4] = 0.0;
    goal_state[5] = 0.0;    

    const ompl::base::GoalPtr goal_ptr(new ManipulatorGoalState(space_information_, goal_state)); 
    
    problem_definition_->addStartState(start_state);    
    problem_definition_->setGoal(goal_ptr);
    
    //planner_->setGoalBias(0.1);
    planner_->setup();
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
    double control_duration = 0.05;
    const char *model_file("/home/hoe01h/catkin_ws/src/phantomx_description/urdf/phantomx_exp.urdf");
    shared::OMPLControlTest ompl_test(model_file,
                                      control_duration,
                                      state_space_dimension, 
                                      control_space_dimension);
    ompl_test.test();
    return 0;
}


