from libintegrate import *
import time
import numpy as np

class InteTest:
    def __init__(self):
        integrate = Integrate()
        controls = v_double()
        
        current_state = v_double()
        
        controls[:] = [1.0, 0.0]
        current_state[:] = [0.0, 0.0, 0.0, 0.0]
        
        t0 = 0.0
        te = 0.3
        delt = 0.03
        
        int_times = v_double()
        int_times[:] = [t0, te, delt]
               
        t0 = time.time()
        integrate.doIntegration(current_state, controls, int_times)
        print "integration took " + str(time.time() - t0) + " seconds"
        res = integrate.getResult()
        result = [res[i] for i in xrange(len(res))]
        print result
        
if __name__ == "__main__":
    InteTest()