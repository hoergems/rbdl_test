from libintegrate import *
import time
import numpy as np

class InteTest:
    def __init__(self):
        integrate = Integrate()
        controls = v_double()
        
        current_state = v_double()
        
        controls[:] = [1.0, 0.0]
        current_state[:] = [0.0, 0.0, 2.0, 0.0]
        
        t0 = 0.0
        te = 0.3
        delt = 0.00003
        
        int_times = v_double()
        int_times[:] = [t0, te, delt]
               
        t0 = time.time()
        A = integrate.getProcessMatrices(current_state)
        Matr_list = [A[i] for i in xrange(len(A))]
        
        A_list = np.array([Matr_list[i] for i in xrange(len(current_state)**2)])
        B_list = np.array([Matr_list[i] for i in xrange(len(current_state)**2, (len(current_state)**2) + 2 * len(current_state))])
        V_list = np.array([Matr_list[i] for i in xrange((len(current_state)**2) + 2 * len(current_state), 
                                                        ((len(current_state)**2) + 4 * len(current_state)))])
        
        A_Matr = A_list.reshape(len(current_state), len(current_state)).T
        B_Matr = B_list.reshape(len(current_state) / 2, len(current_state)).T
        V_Matr = V_list.reshape(len(current_state) / 2, len(current_state)).T
        
        print A_Matr
        print B_Matr
        print V_Matr
        
        result = A_Matr.dot(np.array([0.0, 0.0, 2.0, 0.0])) + B_Matr.dot(np.array([1.0, 0.0]))
        
        print "result " + str(result)
        #integrate.doIntegration(current_state, controls, int_times)
        print "integration took " + str(time.time() - t0) + " seconds"
        #res = integrate.getResult()
        #result = [res[i] for i in xrange(len(res))]
        #print result
        
if __name__ == "__main__":
    InteTest()