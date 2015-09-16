#ifndef MANIPULATOR_GOAL_HPP_
#define MANIPULATOR_GOAL_HPP_
#include <iostream>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>

namespace shared {

    class ManipulatorGoalState : public ompl::base::Goal {
        public:
            ManipulatorGoalState(const ompl::base::SpaceInformationPtr &si,
                                 const ompl::base::ScopedState<> goal_state);
            
            virtual bool isSatisfied(const ompl::base::State *st) const; 

        private:
            ompl::base::SpaceInformationPtr state_space_information_; 

            ompl::base::ScopedState<> goal_state_;          
    };

}

#endif
