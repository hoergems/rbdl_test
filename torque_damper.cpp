#include "torque_damper.hpp"

namespace shared {

TorqueDamper::TorqueDamper(double coulomb, double viscous):
    coulomb_(coulomb),
    viscous_(viscous)
{

}

void TorqueDamper::damp_torques(std::vector<OpenRAVE::dReal> &current_velocities, 
                                std::vector<OpenRAVE::dReal> &torques) {
    double damped_torque = 0;    
    for (size_t i = 0; i < current_velocities.size(); i++) {    
        if (current_velocities[i] != 0.0) {
            double damped_torque = -(coulomb_ * (current_velocities[i] / fabs(current_velocities[i])) + 
                                     viscous_ * current_velocities[i]);
            torques[i] = torques[i] + damped_torque;
        }
    }                                 
}

}
