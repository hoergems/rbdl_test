#include "goal.hpp"

using std::cout;
using std::endl;

namespace shared {

ManipulatorGoalState::ManipulatorGoalState(const ompl::base::SpaceInformationPtr &si,
                                           const ompl::base::ScopedState<> goal_state):
    ompl::base::Goal(si),
    state_space_information_(si),
    goal_state_(goal_state)
{

}

bool ManipulatorGoalState::isSatisfied(const ompl::base::State *st) const {        
    double sum(0.0);
    std::vector<double> vec;    
    for (unsigned int i = 0; i < state_space_information_->getStateDimension() / 2; i++) {
        sum += pow(goal_state_[i] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i], 2);
    }

    double dist(sqrt(sum));    
    if (dist < 0.3) {
        cout << "IS SATISFIED!!!!" << endl;
        return true;
    }    
    return false;
}


}
