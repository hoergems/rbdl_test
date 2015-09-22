#include "ompl_control.hpp"

using std::cout;
using std::endl;

namespace shared {

OMPLControlTest::OMPLControlTest(const std::string &collada_model,
                                 double &control_duration,
                                 double &simulation_step_size):
    control_duration_(control_duration),
    state_space_(nullptr),
    state_space_bounds_(1),
    control_space_(nullptr),
    space_information_(nullptr),
    problem_definition_(nullptr),
    planner_(nullptr),
    state_propagator_(nullptr)    
{    
    
    /***** Initialize OpenRAVE *****/
    OpenRAVE::RaveInitialize(true);
    OpenRAVE::EnvironmentBasePtr environment(OpenRAVE::RaveCreateEnvironment());     
    environment->Load(collada_model);
    environment->StopSimulation();
    
    std::vector<OpenRAVE::RobotBasePtr> robots;
    environment->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots[0];
    
    /***** Setup OMPL *****/
    setup_ompl_(robot, simulation_step_size);
    
    /***** Create the physics engine *****/
    const std::string engine = "ode";
    OpenRAVE::PhysicsEngineBasePtr physics_engine_ = OpenRAVE::RaveCreatePhysicsEngine(environment, engine);
    const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});
    physics_engine_->SetGravity(gravity);
    environment->SetPhysicsEngine(physics_engine_);
    
    boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupOpenRAVEEnvironment(environment, robot);
}

bool OMPLControlTest::setup_ompl_(OpenRAVE::RobotBasePtr &robot, double &simulation_step_size) {
    // The state space consists of joint angles + velocity    
    state_space_dimension_ = robot->GetDOF() * 2;
    control_space_dimension_ = state_space_dimension_ / 2;
    
    state_space_ = boost::make_shared<ompl::base::RealVectorStateSpace>(state_space_dimension_);    
    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);
    control_space_ = boost::make_shared<ompl::control::RealVectorControlSpace>(state_space_, control_space_dimension_);
    
    space_information_ = boost::make_shared<ompl::control::SpaceInformation>(state_space_, control_space_);
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(control_duration_);
     
    problem_definition_ = boost::make_shared<ompl::base::ProblemDefinition>(space_information_);
    planner_ = boost::make_shared<ompl::control::RRT>(space_information_);
    planner_->as<ompl::control::RRT>()->setIntermediateStates(false);
    planner_->setProblemDefinition(problem_definition_); 
    
    state_propagator_ = boost::make_shared<StatePropagator>(space_information_, simulation_step_size);    
    space_information_->setStatePropagator(state_propagator_);
    
    // Set the bounds
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    for (size_t i = 0; i < joints.size(); i++) {
        std::vector<OpenRAVE::dReal> lower_limit;
        std::vector<OpenRAVE::dReal> upper_limit;        
        joints[i]->GetLimits(lower_limit, upper_limit);        
        
        state_space_bounds_.setLow(i, lower_limit[0]);
        state_space_bounds_.setHigh(i, upper_limit[0]);
        
        state_space_bounds_.setLow(i + state_space_dimension_ / 2, -joints[i]->GetMaxVel());
        state_space_bounds_.setHigh(i + state_space_dimension_ / 2, joints[i]->GetMaxVel());
    }
    double torque_bounds = 0.7;
    
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
    // Set the start and goal state
    ompl::base::ScopedState<> start_state(state_space_);
    start_state[0] = 0.0;
    start_state[1] = 0.0;
    start_state[2] = 0.0;
    start_state[3] = 0.0;
    start_state[4] = 0.0;
    start_state[5] = 0.0;
    
    ompl::base::ScopedState<> goal_state(state_space_);
    goal_state[0] = 2.0;
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
        cout << "Length of solution path " << solution_path_->length() << endl << endl;
        std::vector<ompl::base::State*> solution_states_(solution_path_->getStates());
        std::vector<ompl::control::Control*> solution_controls_(solution_path_->getControls());
        for (size_t i = 0; i < solution_states_.size(); i++) {
            cout << "State: ";
            for (size_t j = 0; j < 6; j++) {
                cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] << ", ";
            }
            cout << endl;
            
            if (i < solution_states_.size() - 1) {
                cout << "Control: ";
                for (size_t j = 0; j < 3; j++) {
                    cout << solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j] << ", ";
                }
                cout << endl;
            }
            
        }
    }
}
 
}

int main(int argc, char** argv) {    
    double control_duration = 0.05;
    double simulation_step_size = 0.0005;
    const std::string collada_model("./phantomx_exp2.dae");    
    shared::OMPLControlTest ompl_test(collada_model,
                                      control_duration,
                                      simulation_step_size);
    ompl_test.test();
    OpenRAVE::RaveDestroy();
    return 0;
}


