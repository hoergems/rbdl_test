#include "MotionValidator.hpp"

using std::cout;
using std::endl;

using namespace fcl;

namespace shared {

MotionValidator::MotionValidator(const ompl::base::SpaceInformationPtr &si,
                                 std::shared_ptr<Kinematics> kinematics,                                 
                                 std::vector<std::shared_ptr<Obstacle> > obstacles,
                                 double delta_t,
                                 bool continuous_collision):    
    ompl::base::MotionValidator(si),
    si_(si),
    kinematics_(kinematics),
    obstacles_(obstacles),
    delta_t_(delta_t),
    continuous_collision_(continuous_collision),    
    utils_(),
    link_dimensions_()
{
    
}

bool MotionValidator::checkMotion(const std::vector<double> &s1, 
                                  const std::vector<double> &s2, 
                                  const bool &continuous_collision) const {
	/**for (size_t i = 0; i < s1.size(); i++) {		
		if ((fabs((s2[i] - s1[i]) / delta_t_)) > max_joint_velocity_ + 0.00001) {
			return false;
		}		
	}*/
	
    std::vector<OBB> manipulator_collision_structures_goal = utils_.createManipulatorCollisionStructures(s2,
                                                                                                         link_dimensions_, 
                                                                                                         kinematics_);
    
    for (size_t i = 0; i < obstacles_.size(); i++) {        
        if (!obstacles_[i]->isTraversable()) {
        	if (obstacles_[i]->in_collision(manipulator_collision_structures_goal)) {
        		return false;
        	}
        }        
    }
    
    
    if (continuous_collision) {        
        std::vector<fcl::CollisionObject> manipulator_collision_objects_start = utils_.createManipulatorCollisionObjects(s1, 
                                                                                                                         link_dimensions_,
                                                                                                                         kinematics_);
        std::vector<fcl::CollisionObject> manipulator_collision_objects_goal = utils_.createManipulatorCollisionObjects(s2, 
                                                                                                                        link_dimensions_,
                                                                                                                        kinematics_);
        for (size_t i = 0; i < obstacles_.size(); i++) {
            if (!obstacles_[i]->isTraversable()) {
                for (size_t j = 0; j < manipulator_collision_objects_start.size(); j++) {                	
                	if (obstacles_[i]->in_collision(manipulator_collision_objects_start[j], manipulator_collision_objects_goal[j])) {
                		return false;
                	}
                }
            }
        } 
    }     
      
    return true;
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {    
    std::vector<double> angles1;
    std::vector<double> angles2;    
    for (unsigned int i = 0; i < si_->getStateSpace()->getDimension(); i++) {
        angles1.push_back(s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
        angles2.push_back(s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
        
    return checkMotion(angles1, angles2, continuous_collision_);
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, 
                                  const ompl::base::State *s2, 
                                  std::pair< ompl::base::State *, double > &/*lastValid*/) const {    
    return checkMotion(s1, s2);
}

bool MotionValidator::isValid(const std::vector<double> &s1) const {
    std::vector<OBB> manipulator_collision_structures = utils_.createManipulatorCollisionStructures(s1, 
                                                                                                    link_dimensions_,
                                                                                                    kinematics_);
    for (size_t i = 0; i < obstacles_.size(); i++) {
        if (!obstacles_[i]->getTerrain()->isTraversable()) {        	
        	if (obstacles_[i]->in_collision(manipulator_collision_structures)) {
        		return false;
        	}
        }
    }
    return true;    
}

void MotionValidator::setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles) {
    obstacles_.clear();
    for (size_t i = 0; i < obstacles.size(); i++) {       
        obstacles_.push_back(obstacles[i]);
    }    
}

void MotionValidator::setLinkDimensions(std::vector<std::vector<double>> &link_dimensions) {
    for (size_t i = 0; i < link_dimensions.size(); i++) {
        link_dimensions_.push_back(link_dimensions[i]);
    }    
}
}
