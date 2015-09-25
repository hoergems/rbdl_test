#include "ompl_control.hpp"

using std::cout;
using std::endl;

namespace shared {

OMPLControlTest::OMPLControlTest(const std::string &model_file,
                                 double &control_duration,
                                 double &simulation_step_size):
    damper_(new TorqueDamper(0.0, 1.0)),
    control_duration_(control_duration),
    state_space_(nullptr),
    state_space_bounds_(1),
    control_space_(nullptr),
    space_information_(nullptr),
    problem_definition_(nullptr),
    planner_(nullptr),
    state_propagator_(nullptr),
    env_(nullptr)   
{
    /***** Initialize OpenRAVE *****/
    OpenRAVE::RaveInitialize(true);    
    env_ = OpenRAVE::RaveCreateEnvironment();    

    const std::string module_str("or_urdf_plugin");
    if(!OpenRAVE::RaveLoadPlugin(module_str)) {
        cout << "Failed to load the or_urdf_plugin." << endl;
        return;
    }
    
    OpenRAVE::ModuleBasePtr urdf_module = OpenRAVE::RaveCreateModule(env_, "URDF");
    const std::string cmdargs("");
    env_->AddModule(urdf_module, cmdargs);
    std::stringstream sinput, sout;
    sinput << "load " << model_file;
    if (!urdf_module->SendCommand(sout,sinput)) {
        cout << "Failed to load URDF model" << endl;
    }
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    cout << "len bodies " << bodies.size() << endl;  
    //env_->Load(collada_model);
    env_->StopSimulation();
    
    /**std::vector<OpenRAVE::RobotBasePtr> robots;
    env_->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots[0];*/
    
    OpenRAVE::RobotBasePtr robot = getRobot();

    const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks());
    cout << "link 0 name " << links[0]->GetName() << endl;
    //links[0]->Enable(false);
    links[0]->SetStatic(true);
    cout << "is static " << links[0]->IsStatic() << endl;
    
    /***** Setup OMPL *****/
    setup_ompl_(robot, simulation_step_size);
    
    /***** Create the physics engine *****/
    const std::string engine = "ode";
    OpenRAVE::PhysicsEngineBasePtr physics_engine_ = OpenRAVE::RaveCreatePhysicsEngine(env_, engine);
    const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});
    //const OpenRAVE::Vector gravity({0.0, 0.0, -5.81});
    physics_engine_->SetGravity(gravity);
    env_->SetPhysicsEngine(physics_engine_);
    
    boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupOpenRAVEEnvironment(env_, robot);
}

OpenRAVE::EnvironmentBasePtr OMPLControlTest::getEnvironment() {
    return env_;
}

OpenRAVE::RobotBasePtr OMPLControlTest::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(bodies[0]);
    return robot;
}

ompl::control::ControlSamplerPtr OMPLControlTest::allocUniformControlSampler_(const ompl::control::ControlSpace *control_space) {    
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(control_space));
}

bool OMPLControlTest::setup_ompl_(OpenRAVE::RobotBasePtr &robot, double &simulation_step_size) {
    // The state space consists of joint angles + velocity    
    state_space_dimension_ = robot->GetDOF() * 2;
    control_space_dimension_ = state_space_dimension_ / 2;
    
    state_space_ = boost::make_shared<ompl::base::RealVectorStateSpace>(state_space_dimension_);    
    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);
    control_space_ = boost::make_shared<ControlSpace>(state_space_, control_space_dimension_);
    //control_space_->setControlSamplerAllocator(boost::bind(&OMPLControlTest::allocUniformControlSampler_, this, _1));
    ompl::control::ControlSamplerPtr ptr_ = control_space_->allocControlSampler();
    ompl::control::ControlSamplerPtr ptr2_ = control_space_->allocDefaultControlSampler();
    
    //control_space_->setControlSamplerAllocator(ompl::control::ControlSamplerAllocator(boost::bind(&OMPLControlTest::allocUniformControlSampler_, this, _1)));
    //control_space_->setControlSamplerAllocator(boost::function<ompl::control::ControlSamplerPtr(const ompl::control::ControlSpace *)>(&OMPLControlTest::allocUniformControlSampler_, this, _1));
    
    space_information_ = boost::make_shared<ompl::control::SpaceInformation>(state_space_, control_space_);
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(control_duration_);
     
    problem_definition_ = boost::make_shared<ompl::base::ProblemDefinition>(space_information_);
    planner_ = boost::make_shared<ompl::control::RRT>(space_information_);
    planner_->as<ompl::control::RRT>()->setIntermediateStates(false);
    planner_->setProblemDefinition(problem_definition_); 
    
    state_propagator_ = boost::make_shared<StatePropagator>(space_information_, 
                                                            simulation_step_size,
                                                            damper_);    
    space_information_->setStatePropagator(state_propagator_);
    
    // Set the bounds
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);
    double torque_bounds = 20.0;    
    for (size_t i = 0; i < joints.size(); i++) {
        std::vector<OpenRAVE::dReal> lower_limit;
        std::vector<OpenRAVE::dReal> upper_limit;        
        joints[i]->GetLimits(lower_limit, upper_limit);        
        
        // Set the joints position bounds
        cout << "pos low " << lower_limit[0] << endl;
        cout << "pos high " << upper_limit[0] << endl;
        state_space_bounds_.setLow(i, lower_limit[0]);
        state_space_bounds_.setHigh(i, upper_limit[0]);

        // Set the joints velocity bounds 
        cout << "vel low " << -joints[i]->GetMaxVel() << endl;
        cout << "vel high " << joints[i]->GetMaxVel() << endl;       
        state_space_bounds_.setLow(i + state_space_dimension_ / 2, -joints[i]->GetMaxVel());
        state_space_bounds_.setHigh(i + state_space_dimension_ / 2, joints[i]->GetMaxVel());

        cout << "torque low " << -torque_bounds << endl;
        cout << "torque high " << torque_bounds << endl;
        control_bounds.setLow(i, -torque_bounds);
        control_bounds.setHigh(i, torque_bounds);
        //torque_bounds = torque_bounds - 0.1;
    }    
    
    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_space_bounds_);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);
    return true;
}

bool OMPLControlTest::isValid(const ompl::base::State *state) {
    bool valid = state_space_->as<ompl::base::RealVectorStateSpace>()->satisfiesBounds(state);
    //cout << "State valid: ";
    /**for (unsigned int i = 0; i < state_space_dimension_; i++) {
        cout << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << " ";
    }
    if (valid) {
        cout << "is valid";
    }
    else {
        cout << "not valid";
    }
    cout << endl;*/
    return valid;
}

bool OMPLControlTest::solve_() {
    bool solved = false;
    bool hasExactSolution = false;
    boost::timer t;
    while (!solved && !hasExactSolution) {
        solved = planner_->solve(20.0);
        
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
    ompl::base::ScopedState<> goal_state(state_space_);
    for (unsigned int i = 0; i < state_space_dimension_; i++) {
        start_state[i] = 0.0;
        goal_state[i] = 0.0;
    }
    
    goal_state[0] = 1.0;
    goal_state[1] = 1.0;

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
            for (size_t j = 0; j < state_space_dimension_; j++) {
                cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] << ", ";
            }
            cout << endl;
            
            if (i < solution_states_.size() - 1) {
                cout << "Control: ";
                for (size_t j = 0; j < state_space_dimension_ / 2; j++) {
                    cout << solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j] << ", ";
                }
                cout << endl;
            }
            
        }
    }
}

void OMPLControlTest::damp_torques_(std::vector<OpenRAVE::dReal> &current_velocities,
                                    std::vector<OpenRAVE::dReal> &torques) {
    double c(0.0);
    double v(1.0);
    for (size_t i = 0; i < current_velocities.size(); i++) {    
        if (current_velocities[i] != 0.0) {
            torques[i] = -(c * (current_velocities[i] / fabs(current_velocities[i])) + 
                           v * current_velocities[i]);
        }
        else {
            torques[i] = 0.0;
        }
        
    }
}

void OMPLControlTest::testPhysics() {
    cout << "Testing physics" << endl;    
    OpenRAVE::RobotBasePtr robot = getRobot();
    
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());    
    std::vector<OpenRAVE::dReal> current_vel;
    std::vector<OpenRAVE::dReal> damped_torques;
    for (size_t i = 0; i < joints.size(); i++) {
        damped_torques.push_back(0);
    }
    
    double delta_t(0.0001);
    
    while(true) {
        robot->GetDOFVelocities(current_vel);
        damper_->damp_torques(current_vel, damped_torques);
        cout << "current vel: ";        
        for (size_t k = 0; k < joints.size(); k++) {
            cout << current_vel[k] << " ";
            if (k == 0) {                  
                  //cout << "WHAT " << damped_torques[k] << ", " << current_vel[k] << endl;
                  damped_torques[k] = damped_torques[k] + 20.0;                  
            }   
            const std::vector<OpenRAVE::dReal> torques({damped_torques[k]});
            joints[k]->AddTorque(torques);
        }
        env_->StepSimulation(delta_t);        
        env_->StopSimulation();
        usleep(1000000 * delta_t);
        cout << endl;
    }
}
 
}

int main(int argc, char** argv) {    
    double control_duration = 0.05;
    double simulation_step_size = 0.005;
    const std::string model_file("./lbr_iiwa/urdf/lbr_iiwa_meshfree.urdf");    
    shared::OMPLControlTest ompl_test(model_file,
                                      control_duration,
                                      simulation_step_size);
    shared::ViewerTest viewer;
    OpenRAVE::EnvironmentBasePtr env = ompl_test.getEnvironment(); 
    //viewer.testView(env);    
    //ompl_test.testPhysics();    
    ompl_test.test();
    //OpenRAVE::RaveDestroy();
    return 0;
}


