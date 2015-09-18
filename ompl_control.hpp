#ifndef OMPL_CONTROL_TEST_HPP_
#define OMPL_CONTROL_TEST_HPP_
#include <iostream>
#include <boost/timer.hpp>
#include <boost/thread.hpp>
#include "state_propagator.hpp"
#include "goal.hpp"
#include <openrave-core.h>
#include <openrave/environment.h>

#include <ompl/control/ControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>


namespace shared {

    

    class OMPLControlTest {
        public:
        	OMPLControlTest(const char *model_file,
                                double &control_duration,
                                unsigned int &state_space_dimension,
                                unsigned int &control_space_dimension);
        	
        	bool isValid(const ompl::base::State *state);
        	
        	void test();

        private:
                // The filename of the urdf model
                const char *model_file_;

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

                // Solve the motion planning problem
                bool solve_();

                bool setup_bounds_();
    };
}

#endif
