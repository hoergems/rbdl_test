#ifndef OMPL_CONTROL_TEST_HPP_
#define OMPL_CONTROL_TEST_HPP_
#include <iostream>
#include <boost/timer.hpp>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include "state_propagator.hpp"
#include "goal.hpp"
#include "viewer.hpp"
#include "torque_damper.hpp"
#include "control_space.hpp"
#include <openrave-core.h>
#include <openrave/environment.h>

#include <ompl/control/ControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
//#include <ompl/base/StatePropagatorPtr.h>
#include <ompl/control/StatePropagator.h>


namespace shared {

    typedef boost::shared_ptr<ompl::control::PathControl> PathControlPtr;

    class OMPLControlTest {
        public:
        	OMPLControlTest(const std::string &model_file,
                                double &control_duration,
                                double &simulation_step_size,
                                double &coulomb,
                                double &viscous,
                                bool &linear_propagation,
                                bool &verbose);
                            
                ~OMPLControlTest() { OpenRAVE::RaveDestroy(); }
        	
        	bool isValid(const ompl::base::State *state);
        	
        	PathControlPtr test(double &time_limit);

                OpenRAVE::EnvironmentBasePtr getEnvironment();

                OpenRAVE::RobotBasePtr getRobot();

                void testPhysics(double &simulation_step_size, double &coulomb, double &viscous);

                void viewControls(PathControlPtr &controls,
                                  double &simulation_step_size);
                
                void testNormalDist(double &control_duration,
                		            double &simulation_step_size,
                		            double &coulomb,
                		            double &viscous);

        private:
                boost::shared_ptr<TorqueDamper> damper_;
                
                double accepted_ = 0.0;
                
                double rejected_ = 0.0;

                double control_duration_;

                // Dimension of the state space
                unsigned int state_space_dimension_;

                // Dimension of the control space
                unsigned int control_space_dimension_;

                // The state space
                ompl::base::StateSpacePtr state_space_;

                // The bounds of the state space
                ompl::base::RealVectorBounds state_space_bounds_;

                // The control space
                ompl::control::ControlSpacePtr control_space_;

                // The space information
                ompl::control::SpaceInformationPtr space_information_;

                // The problem definition
                ompl::base::ProblemDefinitionPtr problem_definition_;

                // The planner
                ompl::base::PlannerPtr planner_;
                
                ompl::control::StatePropagatorPtr state_propagator_;

                OpenRAVE::EnvironmentBasePtr env_;

                // Solve the motion planning problem
                bool solve_(double &time_limit);
                
                bool setup_ompl_(OpenRAVE::RobotBasePtr &robot, 
                		         double &simulation_step_size,
                		         bool &linear_propagation,
                		         bool &verbose);
                                   
                ompl::control::ControlSamplerPtr allocUniformControlSampler_(const ompl::control::ControlSpace *control_space);
                
                bool verbose_;
    };
}

#endif
