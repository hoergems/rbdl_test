from libdynamic_planner import *

class DynTest:
    def __init__(self):
        control_duration = 0.005
        simulation_step_size = 0.005
        coulomb = 0.0
        viscous = 1.0
        linear_propagation = False
        verbose = True
        
        ompl_control = OMPLControl("test.urdf",
                                   control_duration,
                                   simulation_step_size,
                                   coulomb,
                                   viscous,
                                   linear_propagation,
                                   verbose)
        print "done"

if __name__ == "__main__":
    DynTest()
    