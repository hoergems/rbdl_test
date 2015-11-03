from libintegrate import *
import time
import numpy as np

class InteTest:
    def __init__(self):
        integrate = Integrate()
        thetas_star = v_double()
        dot_thetas_star = v_double()
        rho_star = v_double()
        
        current_state = v_double()
        
        
        thetas_star[:] = [0.0, 0.0]
        dot_thetas_star[:] = [0.0, 0.0]
        rho_star[:] = [2.0, 0.0]
        current_state[:] = [0.0, 0.0, 1.0, 1.0]
        
        t0 = 0.0
        te = 0.1
        delt = 0.001
        
        int_times = v_double()
        int_times[:] = [t0, te, delt]
        
        integrate.setup(thetas_star, dot_thetas_star, rho_star)        
        t0 = time.time()
        integrate.doIntegration(current_state, int_times)
        print "integration took " + str(time.time() - t0) + " seconds"
        res = integrate.getResult()
        result = [res[i] for i in xrange(len(res))]
        print result
        
if __name__ == "__main__":
    InteTest()