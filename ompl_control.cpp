#include "ompl_control.hpp"
#include <random>
#include <fstream>
#include <iterator>
#include <stdio.h>

using std::cout;
using std::endl;

namespace shared {

OMPLControlTest::OMPLControlTest(const std::string &model_file,
                                 double &control_duration,
                                 double &simulation_step_size,
                                 double &coulomb,
                                 double &viscous,
                                 bool &linear_propagation):    
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
        return;
    }
    cout << "Succesfully loaded URDF model" << endl;
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    env_->StopSimulation();
    
    OpenRAVE::RobotBasePtr robot = getRobot();

    const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks());    
    links[0]->SetStatic(true);
    cout << "is static " << links[0]->IsStatic() << endl;
    
    /***** Setup OMPL *****/
    cout << "setting up ompl" << endl;
    setup_ompl_(robot, simulation_step_size, linear_propagation);
    cout << "ompl set up" << endl;
    
    /***** Create the physics engine *****/
    const std::string engine = "ode";
    OpenRAVE::PhysicsEngineBasePtr physics_engine_ = OpenRAVE::RaveCreatePhysicsEngine(env_, engine);
    //const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});
    const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});
    physics_engine_->SetGravity(gravity);
    env_->SetPhysicsEngine(physics_engine_);
    cout << "setting up state propagator" << endl;
    boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupOpenRAVEEnvironment(env_, 
    		                                                                                 robot,
    		                                                                                 coulomb,
    		                                                                                 viscous);
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
	cout << "hello2" << endl;
	return nullptr;
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(control_space));
}

bool OMPLControlTest::setup_ompl_(OpenRAVE::RobotBasePtr &robot, 
		                          double &simulation_step_size,
		                          bool &linear_propagation) {
    // The state space consists of joint angles + velocity    
    state_space_dimension_ = robot->GetDOF() * 2;
    control_space_dimension_ = state_space_dimension_ / 2;
    cout << "robot dof " << robot->GetDOF() << endl;
    cout << "state space dimension " << state_space_dimension_ << endl;
    cout << "control_space_dimension " << control_space_dimension_ << endl;
    state_space_ = boost::make_shared<ompl::base::RealVectorStateSpace>(state_space_dimension_);    
    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);
    control_space_ = boost::make_shared<ControlSpace>(state_space_, control_space_dimension_);
    
    space_information_ = boost::make_shared<ompl::control::SpaceInformation>(state_space_, control_space_);
    space_information_->setStateValidityChecker(boost::bind(&OMPLControlTest::isValid, this, _1));
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(control_duration_);
     
    problem_definition_ = boost::make_shared<ompl::base::ProblemDefinition>(space_information_);
    planner_ = boost::make_shared<ompl::control::RRT>(space_information_);
    planner_->as<ompl::control::RRT>()->setIntermediateStates(true);
    planner_->as<ompl::control::RRT>()->setGoalBias(0.1);
    planner_->setProblemDefinition(problem_definition_); 
    
    bool verbose = false;
    state_propagator_ = boost::make_shared<StatePropagator>(space_information_, 
                                                            simulation_step_size,                                                            
                                                            linear_propagation,
                                                            verbose);    
    space_information_->setStatePropagator(state_propagator_);
    
    // Set the bounds
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);
    //double torque_bounds = 20.0;
        
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

        cout << "torque low " << -joints[i]->GetMaxTorque() << endl;
        cout << "torque high " << joints[i]->GetMaxTorque() << endl;
        control_bounds.setLow(i, -joints[i]->GetMaxTorque());
        control_bounds.setHigh(i, joints[i]->GetMaxTorque());
        //torque_bounds = torque_bounds - 0.1;
    }    
    cout << "what " << joints.size() << endl;
    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_space_bounds_);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);
    sleep(1);
    
    cout << "return true" << endl;
    
    return true;
    
}

bool OMPLControlTest::isValid(const ompl::base::State *state) {	
    bool valid = state_space_->as<ompl::base::RealVectorStateSpace>()->satisfiesBounds(state);
    if (valid) {
    	accepted_ = accepted_ + 1.0;
    }
    else {
    	rejected_ = rejected_ + 1.0;
    }    
    /**cout << "state: ";
    for (size_t i = 0; i < 6; i++) {
        cout << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << " ";
    }
    cout << "valid " << valid << endl;*/
    return valid;
}

bool OMPLControlTest::solve_(double &time_limit) {
    bool solved = false;
    bool hasExactSolution = false;    
    while (!solved && !hasExactSolution) {
        solved = planner_->solve(time_limit);
        
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

PathControlPtr OMPLControlTest::test(double &time_limit) {
    // Set the start and goal state
    ompl::base::ScopedState<> start_state(state_space_);
    ompl::base::ScopedState<> goal_state(state_space_);
    for (unsigned int i = 0; i < state_space_dimension_; i++) {
        start_state[i] = 0.0;
        goal_state[i] = 0.0;
    }
    
    goal_state[0] = M_PI / 2.0;
    goal_state[1] = 0.0;

    const ompl::base::GoalPtr goal_ptr(new ManipulatorGoalState(space_information_, goal_state)); 
    
    problem_definition_->addStartState(start_state);    
    problem_definition_->setGoal(goal_ptr);
    
    //planner_->setGoalBias(0.1);
    planner_->setup();
    bool solved = false;
    
    boost::timer t;
    solved = solve_(time_limit);
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
        PathControlPtr solution_path_ = 
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
        cout << "Solution found in " << t.elapsed() << "seconds" << endl;
        cout << "accepted " << accepted_ << endl;
        cout << "rejected " << rejected_ << endl;
        return solution_path_; 
    }
}

void OMPLControlTest::viewControls(PathControlPtr &controls,
                                   double &simulation_step_size) {
    shared::ViewerTest viewer;
    viewer.testView(env_);
    OpenRAVE::RobotBasePtr robot = getRobot();
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    
    unsigned int control_size = controls->getControls().size();
    unsigned int num_control_steps = 0;

    std::vector<OpenRAVE::dReal> damped_torques;
    std::vector<OpenRAVE::dReal> input_torques;
    std::vector<OpenRAVE::dReal> current_vel;

    std::vector<OpenRAVE::dReal> start_state;
    std::vector<OpenRAVE::dReal> start_vel;

    for (size_t i = 0; i < joints.size(); i++) {
        input_torques.push_back(0.0);
        damped_torques.push_back(0.0);
        start_state.push_back(0.0);
        start_vel.push_back(0.0);
    }   
    
    while (true) { 
        robot->SetDOFValues(start_state);
        robot->SetDOFVelocities(start_vel); 
        sleep(1);
	    for (size_t i = 0; i < control_size; i++) {
            num_control_steps = controls->getControlDuration(i) / simulation_step_size;
	        for (size_t j = 0; j < num_control_steps; j++) {
		        robot->GetDOFVelocities(current_vel);
		        damper_->damp_torques(current_vel, damped_torques);
		        for (size_t k = 0; k < joints.size(); k++) {
		            input_torques[k] = controls->getControls()[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[k] +
		                               damped_torques[k];
		            const std::vector<OpenRAVE::dReal> torques({input_torques[k]});
		            joints[k]->AddTorque(torques);
		        } 
		    
	            env_->StepSimulation(simulation_step_size);        
		        env_->StopSimulation();
		        usleep(1000000 * simulation_step_size);		        
	        }
	        //usleep(1000000 * 0.5);
	    }
        sleep(1); 
    }
}

void OMPLControlTest::testNormalDist(double &control_duration,
		                             double &simulation_step_size,
		                             double &coulomb,
		                             double &viscous) {
	remove("./somefile.txt");
	
	OpenRAVE::RobotBasePtr robot = getRobot();
	//damper = std::make_shared<TorqueDamper>(coulomb, viscous);
	boost::shared_ptr<TorqueDamper> damper(new TorqueDamper(coulomb, viscous));
	    
	const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());    
	std::vector<OpenRAVE::dReal> current_vel;
	std::vector<OpenRAVE::dReal> damped_torques({0.0, 0.0});
	//std::vector<OpenRAVE::dReal> desired_torques({15.0, 0.0, 0.0});
	std::vector<OpenRAVE::dReal> desired_torques;
	std::vector<OpenRAVE::dReal> input_torques({0.0, 0.0});
	std::vector<OpenRAVE::dReal> start_state({0.0, 0.0});
	std::vector<OpenRAVE::dReal> start_vel({0.0, 0.0});
	std::vector<std::vector<OpenRAVE::dReal>> res_states;
	std::vector<OpenRAVE::dReal> res_state;
	std::random_device rd;
	std::mt19937 gen(rd());
	
	std::normal_distribution<double> dist(0, 200);	
	
	for (unsigned int i = 0; i < 1000; i++) {
		cout << "Sim run " << i << endl;
		desired_torques.clear();
		res_state.clear();
		
		robot->SetDOFValues(start_state);
	    robot->SetDOFVelocities(start_vel);
		
	    cout << "desired_torques ";
		for (size_t k = 0; k < joints.size(); k++) {
			desired_torques.push_back(dist(gen));
			//desired_torques.push_back(0.0);
			cout << desired_torques[k] << ", ";
		}
		cout << endl;
		int num_steps = control_duration / simulation_step_size;
		for (unsigned int j = 0; j < num_steps; j++) {
			robot->GetDOFVelocities(current_vel);        
		    damper->damp_torques(current_vel, damped_torques);		    
			for (size_t k = 0; k < joints.size(); k++) {            
			    input_torques[k] = desired_torques[k] + damped_torques[k];            
			    const std::vector<OpenRAVE::dReal> torques({input_torques[k]});
			    joints[k]->AddTorque(torques);
		    }		
			env_->StepSimulation(simulation_step_size);        
			env_->StopSimulation();
			
		}
		
		robot->GetDOFValues(res_state);
		res_states.push_back(res_state);
	} 
	
	std::ofstream f("./somefile.txt");
	
	for (size_t i = 0; i < res_states.size(); i++) {
		for (size_t k = 0; k < res_states[i].size(); k++) {
			f << res_states[i][k] << " ";
		}
		f << "\n";
	}
}

void OMPLControlTest::testPhysics(double &simulation_step_size, double &coulomb, double &viscous) {
    shared::ViewerTest viewer;
    viewer.testView(env_);
    cout << "Testing physics" << endl;    
    OpenRAVE::RobotBasePtr robot = getRobot();
    boost::shared_ptr<TorqueDamper> damper(new TorqueDamper(coulomb, viscous));
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());    
    std::vector<OpenRAVE::dReal> current_vel;
    std::vector<OpenRAVE::dReal> damped_torques;
    //std::vector<OpenRAVE::dReal> desired_torques({15.0, 0.0, 0.0});
    std::vector<OpenRAVE::dReal> desired_torques({0.0, 0.1});
    std::vector<OpenRAVE::dReal> input_torques({0.0, 0.0,});
    std::vector<OpenRAVE::dReal> start_state({0.0, 0.0});
    std::vector<OpenRAVE::dReal> start_vel({0.0, 0.0});

    robot->SetDOFValues(start_state);
    robot->SetDOFVelocities(start_vel); 

    int b = 0;
    for (size_t i = 0; i < joints.size(); i++) {
        damped_torques.push_back(0);
    }
    
    while(true) {
        b++;
        robot->GetDOFVelocities(current_vel); 
        cout << "curr vel size " << current_vel.size() << endl;
        cout << "damped_torques size " << damped_torques.size() << endl;
        damper->damp_torques(current_vel, damped_torques);
        
        for (size_t k = 0; k < joints.size(); k++) {            
            input_torques[k] = desired_torques[k] + damped_torques[k];            
            const std::vector<OpenRAVE::dReal> torques({input_torques[k]});
            joints[k]->AddTorque(torques);
        }
        env_->StepSimulation(simulation_step_size);        
        env_->StopSimulation();
        usleep(1000000 * simulation_step_size);        
    }
}
 
}

int main(int argc, char** argv) {
    double coulomb = 0.0;
    double viscous = 1.0;    
    //double control_duration = 0.07;
    double control_duration = 0.1;
    double simulation_step_size = 0.001;    
    double time_limit = 50.0;
    bool linear_propagation = false;
    //const std::string model_file("./lbr_iiwa/urdf/lbr_iiwa_meshfree.urdf");
    const std::string model_file("test.urdf");
    shared::OMPLControlTest ompl_test(model_file,
                                      control_duration,
                                      simulation_step_size,
                                      coulomb,
                                      viscous,
                                      linear_propagation);
    ompl_test.testNormalDist(control_duration, simulation_step_size, coulomb, viscous);    
    //OpenRAVE::EnvironmentBasePtr env = ompl_test.getEnvironment(); 
    
    //ompl_test.testPhysics(simulation_step_size, coulomb, viscous);    
    //shared::PathControlPtr controls = ompl_test.test(time_limit);
    //ompl_test.viewControls(controls,
                           //simulation_step_size);
    //OpenRAVE::RaveDestroy();
    return 0;
}


