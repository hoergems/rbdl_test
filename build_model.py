from librobot import *
from sympy import *
import numpy as np
import time
import os
import sys
import argparse
from sympy.printing import print_ccode
from sympy.abc import x
import mpmath as mp

from scipy.integrate import ode, odeint
from sympy.integrals.trigonometry import trigintegrate
from gi.overrides.keysyms import R10

class Test:
    def __init__(self, model, simplifying, buildcpp):
        self.simplifying = simplifying
        self.parse_urdf(model)
        g = Matrix([[0],
                    [0],
                    [9.81]])       
        """
        Get the Jacobians of the links expressed in the robot's base frame
        """        
        print "Calculating Jacobian matrices"
        t_start = time.time()
        Jvs, Ocs = self.get_link_jacobians(self.joint_origins, self.inertial_poses, self.joint_axis, self.q)
        #print Jvs
        #print Ocs
        
        M_is = self.construct_link_inertia_matrices(self.link_masses, self.Is)        
        print "Calculating inertia matrix"
        M = self.calc_inertia_matrix(Jvs, M_is)
        
        if self.simplifying:
            M = trigsimp(M)
            M = nsimplify(M, tolerance=1e-7)        
        print "Calculating coriolis matrix"
        C = self.calc_coriolis_matrix(self.q, self.qdot, M)
        if self.simplifying:
            C = trigsimp(C)             
        print "Calculating normal forces" 
        viscous = 1.0
        N = self.calc_generalized_forces(self.q,
                                         self.qdot, 
                                         Ocs, 
                                         self.link_masses, 
                                         g,
                                         viscous)        
        if self.simplifying:      
            N = trigsimp(N)        
        print "Inverting inertia matrix..."
        t0 = time.time()
        M_inv = M.inv()
        print "Inversion took " + str(time.time() - t0) + " seconds"        
        #f, M_inv = self.get_dynamic_model(M, C, N, self.q, self.qdot, self.rho, self.zeta)
        print "Build taylor approximation" 
        steady_states = self.get_steady_states()
        print "Get partial derivatives"           
        #A, B, V = self.partial_derivatives2(f)        
        A, B, V = self.partial_derivatives(M_inv, C, N)        
        
        '''if self.simplifying:
            A = simplify(A)
            B = simplify(B)
            V = simplify(V)'''       
        print "Clean cpp code"
        self.clean_cpp_code()
        print "Gen cpp code"
        self.gen_cpp_code_steady_states(steady_states) 
        print "Steady states code generated"
        print "Generate cpp code for linearized model..."
        self.initial = [0.0, 0.0, 0.0, 0.0]
        self.input = [1.0, 0.0]   
        for i in xrange(len(steady_states)):
            A, B, V = self.substitude_steady_states2(A, B, V, steady_states[i])            
            #self.test_ode(A, B, steady_states[i]) 
            #self.test_int(A, B, steady_states[i])
            self.gen_cpp_code2(A, "A" + str(i))
            self.gen_cpp_code2(B, "B" + str(i))
            self.gen_cpp_code2(V, "V" + str(i))
        print "Building model took " + str(time.time() - t_start) + " seconds"  
        if buildcpp:
            print "Build c++ code..."
            cmd = "cd build && cmake .. && make -j8"           
            os.system(cmd)
        print "Done"
        
    def su(self, A1, A2):
        for i in xrange(len(self.q) - 1):
            A1 = A1.subs(self.q[i], 0.0)
            A2 = A2.subs(self.q[i], 0.0)
            
            A1 = A1.subs(self.qdot[i], 0.0)
            A2 = A2.subs(self.qdot[i], 0.0)
        print A1
        print "=============="
        print A2
        sleep
        
    def test_ode(self, A, B, steady_state):
        self.A = A
        self.B = B
        self.steady_state = steady_state
        t = np.linspace(0.0, 0.03, 2)
        eq = odeint(self.funct, self.initial, t)
        #print "calc took " + str(time.time() - t0) + " seconds"
        print "================="
        print eq
        sleep
        
    def funct(self, x, t):        
        x1 = Matrix([[self.q[i]] for i in xrange(len(self.q) - 1)])
        x2 = Matrix([[self.qdot[i]] for i in xrange(len(self.qdot) - 1)])        
        x_star = x1.col_join(x2)
        x = Matrix([[x[0]],
                    [x[1]],
                    [x[2]],
                    [x[3]]])
        
        
        
        rho = Matrix([[self.input[i]] for i in xrange(len(self.input))])
        rho_star = Matrix([[0.0] for i in xrange(len(self.input))])
        for key in self.steady_state.keys():
            x_star = x_star.subs(key, self.steady_state[key])
        
        x_star = x_star.subs(self.q[0], self.initial[0])
        x_star = x_star.subs(self.q[1], self.initial[1])
        
        A = self.A.subs(self.q[0], x_star[0])
        A = self.A.subs(self.q[1], x_star[1])
        B = self.B.subs(self.q[0], x_star[0])
        B = self.B.subs(self.q[1], x_star[1])
                
        delta_x = x - x_star
        delta_rho = rho - rho_star
        sol = A * delta_x + B * rho
        print "state " + str(x) 
        print "A " + str(A)
        print "B " + str(B)
        
        #sleep
        
        return [sol[i] for i in xrange(len(sol))]
    
    def test_power_series(self, A, T, depth=15):
        A_t = -A * T
        A_i = A_t
        series = eye(4)                
        for i in xrange(1, depth):
            print i
            series = series + A_i / mp.factorial(i + 1)
            A_i = A_i * A_t        
        return T * series        
        
    def test_int(self, A, B, steady_state):        
        x_0 = Matrix([[self.initial[i]] for i in xrange(len(self.initial))])
        rho = Matrix([[self.input[i]] for i in xrange(len(self.input))])
        t = symbols("t") 
        t_e = 0.3        
        print "Calc matrix exponentials"
        
        
        A = A.subs([(self.q[i], self.initial[i]) for i in xrange(len(self.q) - 1)])
        A = A.subs([(self.qdot[i], self.initial[i + len(self.qdot) - 1]) for i in xrange(len(self.qdot) - 1)])
        B = B.subs([(self.q[i], self.initial[i]) for i in xrange(len(self.q) - 1)])
        B = B.subs([(self.qdot[i], self.initial[i + len(self.qdot) - 1]) for i in xrange(len(self.qdot) - 1)])
        
        print "A:" 
        print A
        print "B:"
        print N(B)
        
        
        A_exp1 = exp(t_e * A)        
        A_exp2 = exp(-t * A)        
        
        #A_exp1 = A_exp1.subs([(self.q[i], self.initial[i]) for i in xrange(len(self.q))])
        #A_exp1 = A_exp1.subs([(self.qdot[i], self.initial[i + len(self.qdot)]) for i in xrange(len(self.qdot))])
        #A_exp2 = A_exp2.subs([(self.q[i], self.initial[i]) for i in xrange(len(self.q))])
        #A_exp2 = A_exp2.subs([(self.qdot[i], self.initial[i + len(self.qdot)]) for i in xrange(len(self.qdot))])
        
        print "calculate integral"
        #integral = integrate(A_exp2, (t, 0, t_e))
        t0 = time.time()
        integral = self.test_power_series(A, t_e, 15)
        print "Integration took " + str(time.time() - t0) + " seconds"
        #print integral
        print "===================="
        print "integral " + str(integral)
        f = A_exp1 * x_0 + A_exp1 * integral * B * rho
        f = f.subs(t, t_e)    
        print f
        print N(f)
        print "factorial " + str(mp.factorial(4))
        sleep       
        
    def get_steady_states(self):
        steady_states = []             
        if len(self.q) == 3:
            if self.joint_origins[0][3] != 0.0:
                ss1 = dict()
                ss2 = dict()
                ss1[self.q[0]] = -np.pi / 2.0
                ss1[self.q[1]] = 0.0
                ss1[self.qdot[0]] = 0.0
                ss1[self.qdot[1]] = 0.0                
                    
                ss2[self.q[0]] = np.pi / 2.0                
                ss2[self.q[1]] = 0.0              
                ss2[self.qdot[0]] = 0.0
                ss2[self.qdot[1]] = 0.0                
                steady_states.append(ss1)
                steady_states.append(ss2)                
                print "return 0"
                return steady_states
                
            if self.joint_origins[1][3] != 0.0:                
                ss1 = dict()
                ss2 = dict()
                ss1[self.q[0]] = self.q[0]
                ss1[self.q[1]] = -np.pi / 2.0                
                ss1[self.qdot[0]] = 0.0
                ss1[self.qdot[1]] = 0.0                
                    
                ss2[self.q[0]] = self.q[0]               
                ss2[self.q[1]] = np.pi / 2.0                
                ss2[self.qdot[0]] = 0.0
                ss2[self.qdot[1]] = 0.0                
                steady_states.append(ss1)
                steady_states.append(ss2)                
                print "return 1"
                return steady_states
            else:
                ss = dict()
                ss[self.q[0]] = self.q[0] 
                ss[self.q[1]] = self.q[1]           
                ss[self.qdot[0]] = 0.0
                ss[self.qdot[1]] = 0.0
                print "return 2"
                steady_states.append(ss)                
                return steady_states
        else:
            if self.joint_origins[0][3] != 0.0:
                ss1 = dict()
                ss2 = dict()
                ss1[self.q[0]] = self.q[0]               
                ss1[self.q[1]] = -np.pi / 2.0
                ss1[self.q[2]] = 0.0
                ss1[self.qdot[0]] = 0.0
                ss1[self.qdot[1]] = 0.0
                ss1[self.qdot[2]] = 0.0
                
                ss2[self.q[0]] = self.q[0]                
                ss2[self.q[1]] = np.pi / 2.0
                ss2[self.q[2]] = 0.0
                ss2[self.qdot[0]] = 0.0
                ss2[self.qdot[1]] = 0.0
                ss2[self.qdot[2]] = 0.0
                print "return 3"
                steady_states.append(ss1)
                steady_states.append(ss2)
                return steady_states
            else:
                ss = dict()  
                ss[self.q[0]] = self.q[0]
                ss[self.q[1]] = self.q[1]
                ss[self.q[2]] = self.q[2]             
                ss[self.qdot[0]] = 0.0
                ss[self.qdot[1]] = 0.0
                ss[self.qdot[2]] = 0.0
                print "return 3"
                steady_states.append(ss)
                return steady_states
        
        print "simplifying fs..."
        for i in xrange(len(f)):
            print i
            print f[i, 0]
            f[i, 0] = trigsimp(f[i, 0])
        equations = []
        variables = []        
        for i in xrange(len(f)):
            equations.append(f[i, 0])
        for i in xrange(len(self.q) - 1):
            variables.append(self.q[i])
        for i in xrange(len(self.q) - 1):
            variables.append(self.qdot[i])
        print "solve..."
        steady_states = solve(equations, variables)
        
        print steady_states
        
        sleep        
        return steady_states[0]
        
    
    
        
    def parse_urdf(self, xml_file):
        robot = Robot(xml_file)
        
        link_names = v_string()
        robot.getLinkNames(link_names)
        self.link_names = [link_names[i] for i in xrange(len(link_names))]
        
        joint_names = v_string()
        robot.getJointNames(joint_names)
        self.joint_names = [joint_names[i] for i in xrange(len(joint_names))]                 
        
        joint_type = v_string()
        robot.getJointType(joint_names, joint_type)
        self.joint_types = [joint_type[i] for i in xrange(len(joint_type))]
        
        joint_origins = v2_double()
        robot.getJointOrigin(joint_names, joint_origins)
        self.joint_origins = [Matrix([[joint_origins[i][j]] for j in xrange(len(joint_origins[i]))]) 
                              for i in xrange(len(joint_origins))]               
        
        joint_axis = v2_double()
        robot.getJointAxis(joint_names, joint_axis)
        self.joint_axis = [Matrix([[joint_axis[i][j]] for j in xrange(len(joint_axis[i]))]) for i in xrange(len(joint_axis))]
        
        self.q = []
        self.qdot = []
        self.qstar = []
        self.qdotstar = []
        self.rho = []
        self.rhostar = []
        self.zeta = []
        for i in xrange(len(self.joint_names)):
            #if self.joint_types[i] == "revolute":            
            
            symb_string_q = "x[" + str(i) + "]"
            symb_string_q_dot = "x[" + str(i + len(self.joint_names) - 1) + "]"
            symb_string_q_star = "thetas_star_[" + str(i) + "]"
            symb_string_q_dot_star = "dot_thetas_star_[" + str(i) + "]"
            symb_string_r = "rho[" + str(i) + "]"
            symb_string_r_star = "rhos_star_[" + str(i) + "]" 
            symb_zeta = "zeta[" + str(i) + "]"          
            
            
            self.q.append(symbols(symb_string_q))
            self.qdot.append(symbols(symb_string_q_dot))
            self.rho.append(symbols(symb_string_r))
            self.qstar.append(symbols(symb_string_q_star))
            self.qdotstar.append(symbols(symb_string_q_dot_star))
            self.rhostar.append(symbols(symb_string_r_star))
            self.zeta.append(symbols(symb_zeta))
        inertia_pose = v2_double()
        robot.getLinkInertialPose(link_names, inertia_pose)
        self.inertial_poses = [[inertia_pose[i][j] for j in xrange(len(inertia_pose[i]))] for i in xrange(len(inertia_pose))]
        
        masses = v_double()
        robot.getLinkMasses(link_names, masses)        
        self.link_masses = [masses[i] for i in xrange(len(masses))]
        
        link_inertias = v2_double()
        robot.getLinkInertias(link_names, link_inertias)        
        self.link_inertias = [Matrix([[link_inertias[i][0], link_inertias[i][1], link_inertias[i][2]],
                                      [link_inertias[i][1], link_inertias[i][3], link_inertias[i][4]],
                                      [link_inertias[i][2], link_inertias[i][4], link_inertias[i][5]]]) for i in xrange(len(link_inertias))]
              
        self.Is = [[self.link_inertias[i][0],
                    self.link_inertias[i][4],
                    self.link_inertias[i][8]]for i in xrange(len(self.link_inertias))] 
        
    def gen_cpp_code_steady_states(self, steady_states):
        lines = list(open("integrate.cpp", 'r'))
        temp_lines = []
        
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
            if "void Integrate::setupSteadyStates() const {" in lines[i]:
                 idx1 = i + 1                 
                 breaking = True
            elif "std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> Integrate::getClosestSteadyStateFunctions" in lines[i]:
                idx2 = i - 3                
                if breaking:
                    break
        for i in xrange(len(steady_states)):
            line = "std::vector<double> steady_state_" + str(i) + "({"
            for j in xrange(len(self.q) - 1):                
                if steady_states[i][self.q[j]] != self.q[j]:
                    line += str(steady_states[i][self.q[j]]) + ", "
                else:
                    line += "-1, "
            for j in xrange(len(self.qdot) - 1):
                if steady_states[i][self.qdot[j]] == 0.0:
                    line += "0.0"
                else:
                    line += "-1"
                if not j == len(self.q) - 2:
                    line += ", "
            line += "}); \n"
            line += "steady_states_.push_back(steady_state_" + str(i) +"); \n"
            line += "a_map_.insert(std::make_pair(" + str(i) + ", &Integrate::getA" + str(i) + ")); \n"
            line += "b_map_.insert(std::make_pair(" + str(i) + ", &Integrate::getB" + str(i) + ")); \n"
            line += "v_map_.insert(std::make_pair(" + str(i) + ", &Integrate::getV" + str(i) + ")); \n"
            temp_lines.append(line)        
        del lines[idx1:idx2]
        idx = -1
        for i in xrange(len(lines)):
            if "void Integrate::setupSteadyStates() const {" in lines[i]:
                idx = i        
        lines[idx+1:idx+1] = temp_lines
        os.remove("integrate.cpp")        
        with open("integrate.cpp", 'a+') as f:
            for line in lines:
                f.write(line)
                
    def clean_cpp_code(self):
        lines = list(open("integrate.cpp", 'r'))
        lines_header = list(open("integrate.hpp", 'r'))
        tmp_lines = []
        idx_pairs = []
        
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
            if "MatrixXd Integrate::getA" in lines[i] or "MatrixXd Integrate::getB" in lines[i] or "MatrixXd Integrate::getV" in lines[i]:
                idx1 = i                
                breaking = True
            if "}" in lines[i] and breaking:
                idx_pairs.append((idx1, i))
                idx1 = -1
                breaking = False               
        for i in xrange(len(lines)):
            app = True
            for j in xrange(len(idx_pairs)):
                if i >= idx_pairs[j][0] and i <= idx_pairs[j][1]:
                    app = False
                    break                
            if app:
                tmp_lines.append(lines[i])
        os.remove("integrate.cpp")        
        with open("integrate.cpp", 'a+') as f:
            for line in tmp_lines:
                f.write(line)
                
        tmp_lines = []
        idxs = []
        for i in xrange(len(lines_header)):
            if "MatrixXd getA" in lines_header[i] or "MatrixXd getB" in lines_header[i] or "MatrixXd getV" in lines_header[i]:
                idxs.append(i)
        for i in xrange(len(lines_header)):
            app = True
            for j in xrange(len(idxs)):
                if i == idxs[j]:
                    app = False
            if app:
                tmp_lines.append(lines_header[i])
                
        os.remove("integrate.hpp")        
        with open("integrate.hpp", 'a+') as f:
            for line in tmp_lines:
                f.write(line)
                
        lines = list(open("integrate.cpp", 'r'))
        tmp_lines = []
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
            if "void Integrate::setupSteadyStates() const {" in lines[i]:
                idx1 = i + 1
                breaking = True
            elif "std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> Integrate::getClosestSteadyStateFunctions" in lines[i]:
                idx2 = i - 3                
                if breaking:
                    break                     
        del lines[idx1:idx2]        
        os.remove("integrate.cpp")        
        with open("integrate.cpp", 'a+') as f:
            for line in lines:               
                f.write(line)
                           
     
    def gen_cpp_code2(self, Matr, name):        
        lines = list(open("integrate.cpp", 'r'))
        lines_header = list(open("integrate.hpp", 'r'))
        temp_lines = []
        if Matr.shape[1] != 1:
            temp_lines.append("MatrixXd m(" + str(Matr.shape[0]) + ", " + str(Matr.shape[1]) + "); \n")
        else:
            temp_lines.append("VectorXd m(" + str(Matr.shape[0]) + "); \n")        
        for i in xrange(Matr.shape[0]):
            for j in xrange(Matr.shape[1]):
                temp_lines.append("m(" + str(i) + ", " + str(j) + ") = " + str(ccode(Matr[i, j])) + "; \n")        
        temp_lines.append("return m; \n")
        idx1 = -1
        idx2 = -1
        breaking = False    
        for i in xrange(len(lines)):
            if "Integrate::get" + name + "(const state_type &x) const{" in lines[i]:                
                idx1 = i + 1               
                breaking = True
            elif "}" in lines[i]:
                idx2 = i - 1
                if breaking:
                    break        
        if idx1 == -1:            
            temp_lines.insert(0, "MatrixXd Integrate::get" + name + "(const state_type &x) const{ \n") 
            temp_lines.append("\n")
            temp_lines.append("} \n \n")                     
            lines[len(lines) - 2:len(lines) - 1] = temp_lines            
            
            temp_lines_header = []
            idx = -1
            for i in xrange(len(lines_header)):
                if "private:" in lines_header[i]:                    
                    idx = i
            temp_lines_header.append("MatrixXd get" + str(name) + "(const state_type &x) const; \n")
            lines_header[idx+1:idx+1] = temp_lines_header
                
        else:                  
            del lines[idx1:idx2]
            idx = -1
            for i in xrange(len(lines)):
                if "Integrate::get" + name in lines[i]:
                    idx = i        
            lines[idx:idx] = temp_lines           
        os.remove("integrate.cpp")
        os.remove("integrate.hpp")
        with open("integrate.cpp", 'a+') as f:
            for line in lines:
                f.write(line)
        with open("integrate.hpp", 'a+') as f:
            for line in lines_header:
                f.write(line)    
        
    def get_dynamic_model(self, M, C, N, thetas, dot_thetas, rs, zetas): 
        print "Inverting inertia matrix"              
        t0 = time.time()
        #M_inv = M.inv("LU")
        M_inv = M.inv()      
        print "time to invert: " + str(time.time() - t0)        
        Thetas = Matrix([[thetas[i]] for i in xrange(len(thetas) - 1)])
        Dotthetas = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        Rs = Matrix([[rs[i]] for i in xrange(len(rs) - 1)])
        Zetas = Matrix([[zetas[i]] for i in xrange(len(zetas) - 1)])
        print "Constructing non-linear differential equation"
        m_upper = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        m_lower = 0
        '''if self.simplifying:
            m_lower = trigsimp(-M_inv * trigsimp(C * Dotthetas + N) + M_inv * Rs)           
        else:'''
        m_lower = M_inv * ((Rs + Zetas) - C * Dotthetas - N)
        h = m_upper.col_join(m_lower)        
        return h, M_inv
    
    def substitude_steady_states2(self, A, B, V, steady_states):                            
        for i in xrange(len(self.rho)):
            A = A.subs(self.rho[i], 0)
            B = B.subs(self.rho[i], 0)
            V = V.subs(self.rho[i], 0)
        for i in xrange(len(self.zeta)):
            A = A.subs(self.zeta[i], 0)
            B = B.subs(self.zeta[i], 0)
            V = V.subs(self.zeta[i], 0)                       
        for i in xrange(len(steady_states.keys())):
            A = A.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])            
            B = B.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
            V = V.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
        
        return A, B, V             
        
        A = zeros(len(self.q) - 1)
        print "Claculate A_low..."
        A_low = A1 + A2 + A3        
        A = A.col_join(A_low)            
        
        B = eye(len(self.q) - 1)
        B = B.col_join(B1)
        A = A.row_join(B)        
        
        B = zeros(len(self.q) - 1)
        B = B.col_join(C1)
        return f, A, B
    
    def partial_derivatives(self, M_inv, C, N):        
        r = Matrix([[self.rho[i]] for i in xrange(len(self.rho) - 1)])
        x1 = Matrix([[self.q[i]] for i in xrange(len(self.q) - 1)])
        x2 = Matrix([[self.qdot[i]] for i in xrange(len(self.qdot) - 1)])
        z = Matrix([[self.zeta[i]] for i in xrange(len(self.zeta) - 1)])        
        A1 = M_inv * r
        A2 = M_inv * z
        A3 = M_inv * (-C * x2)
        A4 = M_inv * (-N)
        
        A1_x1 = A1.jacobian([self.q[i] for i in xrange(len(self.q) - 1)])
        A2_x1 = A2.jacobian([self.q[i] for i in xrange(len(self.q) - 1)])
        A3_x1 = A3.jacobian([self.q[i] for i in xrange(len(self.q) - 1)])
        A4_x1 = A4.jacobian([self.q[i] for i in xrange(len(self.q) - 1)])        
        
        A3_x2 = A3.jacobian([self.qdot[i] for i in xrange(len(self.qdot) - 1)])
        A4_x2 = A4.jacobian([self.qdot[i] for i in xrange(len(self.qdot) - 1)])
        
        A1_r = A1.jacobian([self.rho[i] for i in xrange(len(self.rho) - 1)])
        A2_z = A2.jacobian([self.zeta[i] for i in xrange(len(self.rho) - 1)])
        
        A = zeros(len(self.q) - 1).col_join(A1_x1 + A2_x1 + A3_x1 + A4_x1)
        B = eye(len(self.q) - 1).col_join(A3_x2 + A4_x2)
        A = A.row_join(B)
        
        B = zeros(len(self.q) - 1).col_join(A1_r)
        C = zeros(len(self.q) - 1).col_join(A2_z)
        return A, B, C
    
    def partial_derivatives2(self,
                             f):
        A = f.jacobian([self.q[i] for i in xrange(len(self.q) - 1)]) 
        print "Calculated A"
        B = f.jacobian([self.qdot[i] for i in xrange(len(self.qdot) - 1)])
        print "Calculated B"
        C = f.jacobian([self.rho[i] for i in xrange(len(self.rho) - 1)])
        print "Calculated C"
        D = f.jacobian([self.zeta[i] for i in xrange(len(self.zeta) - 1)])
        print "Calculated D"       
        A_r = A.row_join(B)        
        return A_r, C, D
        
    def calc_generalized_forces(self, 
                                thetas, 
                                dot_thetas, 
                                Ocs, 
                                ms, 
                                g,
                                viscous):
        V = 0.0                          
        for i in xrange(len(Ocs)):            
            el = ms[i + 1] * g.transpose() * Ocs[i]                                                
            V += el[0]               
        N = 0
        if self.simplifying:    
            N = Matrix([[trigsimp(diff(V, thetas[i]))] for i in xrange(len(thetas) - 1)]) 
        else:
            N = Matrix([[diff(V, thetas[i])] for i in xrange(len(thetas) - 1)])        
        K = N + Matrix([[viscous * dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        return K      
        
    def calc_coriolis_matrix(self, thetas, dot_thetas, M):        
        C = Matrix([[0.0 for m in xrange(len(thetas) - 1)] for n in xrange(len(thetas) - 1)])
        for i in xrange(len(thetas) - 1):
            for j in xrange(len(thetas) - 1):
                val = 0.0
                for k in xrange(len(thetas) - 1): 
                    if self.simplifying:                                             
                        val += trigsimp(self.calc_christoffel_symbol(i, j, k, thetas, M) * dot_thetas[k])
                    else:                        
                        val += self.calc_christoffel_symbol(i, j, k, thetas, M) * dot_thetas[k]                
                C[i, j] = val                            
        return C   
    
    def calc_christoffel_symbol(self, i, j, k, thetas, M):
        t_i_j_k = 0.0
        if self.simplifying:
            t_i_j_k = 0.5 * (trigsimp(diff(M[i, j], thetas[k])) + 
                             trigsimp(diff(M[i, k], thetas[j])) -
                             trigsimp(diff(M[k, j], thetas[i])))
        else:
            t_i_j_k = 0.5 * (diff(M[i, j], thetas[k]) + 
                             diff(M[i, k], thetas[j]) -
                             diff(M[k, j], thetas[i]))
        return t_i_j_k
    
    def calc_inertia_matrix(self, Jvs, M_is):        
        res = Matrix([[0.0 for n in xrange(len(Jvs))] for m in xrange(len(Jvs))])
        for i in xrange(len(Jvs)):
            if self.simplifying:
                res += trigsimp(Jvs[i].transpose() * M_is[i] * Jvs[i])
            else:
                res += Jvs[i].transpose() * M_is[i] * Jvs[i]               
        return res
    
    def construct_link_inertia_matrices(self, ms, Is):
        M_is = [Matrix([[ms[i], 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, ms[i], 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, ms[i], 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, Is[i][0], 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, Is[i][1], 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, Is[i][2]]]) for i in xrange(len(ms))]
        return M_is[1:len(M_is)-1]
        
    def get_link_jacobians(self, joint_origins, com_coordinates, axis, thetas):
        """
        Vectors from the center of mass to the next joint origin        
        """        
        com_coordinates = [Matrix([[com_coordinates[i][j]] for j in xrange(len(com_coordinates[i]))]) 
                           for i in xrange(len(com_coordinates))]
        m_to_joint_vectors = [Matrix([[joint_origins[i][0]],
                                      [joint_origins[i][1]],
                                      [joint_origins[i][2]]]) - 
                              Matrix([[com_coordinates[i][0]],
                                      [com_coordinates[i][1]],
                                      [com_coordinates[i][2]]]) for i in xrange(1, len(joint_origins))]        
        
        """
        Transformation matrix from the center of masses to the next joint origins
        """
        trans_matrices2 = [self.transform(m_to_joint_vectors[i][0], 
                                          m_to_joint_vectors[i][1], 
                                          m_to_joint_vectors[i][2], 
                                          0.0, 
                                          0.0, 
                                          0.0) for i in xrange(len(m_to_joint_vectors))]
        
        """
        Transformations from the link origins to the center of masses
        """        
        dhcs = [self.transform(com_coordinates[i + 1][0], 
                               com_coordinates[i + 1][1], 
                               com_coordinates[i + 1][2], 
                               joint_origins[i][3] + axis[i][0] * thetas[i], 
                               joint_origins[i][4] + axis[i][1] * thetas[i], 
                               joint_origins[i][5] + axis[i][2] * thetas[i]) for i in xrange(len(joint_origins) -1)]        
        
        """
        O and z of the first joint
        """
        Os = [Matrix([[joint_origins[0][0]],
                      [joint_origins[0][1]],
                      [joint_origins[0][2]]])]        
        zs = [Matrix([[axis[0][0]],
                      [axis[0][1]],
                      [axis[0][2]]])]
        Ocs = []
        zcs = []
        I = Matrix([[1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])
        res = I
        for i in xrange(len(thetas) - 1):
            res *= dhcs[i]            
            col3 = res.col(2)
            col4 = res.col(3)            
            z = Matrix([col3[j] for j in xrange(3)])
            O = Matrix([col4[j] for j in xrange(3)])
            if self.simplifying:
                Ocs.append(trigsimp(O))
            else:
                Ocs.append(O)
            zcs.append(z)
            res = res * trans_matrices2[i]            
            col3 = res.col(2)
            col4 = res.col(3)            
            z = Matrix([col3[j] for j in xrange(3)])
            O = Matrix([col4[j] for j in xrange(3)])
            if self.simplifying:
                Os.append(trigsimp(O))
            else:
                Os.append(O)
            zs.append(z)
        Jvs = []
        for i in xrange(len(thetas) - 1):
            Jv = Matrix([[0.0 for m in xrange(len(thetas) - 1)] for n in xrange(6)])
            for k in xrange(i + 1):
                r1 = 0.0
                if self.simplifying:
                    r1 = trigsimp(Matrix(zcs[i].cross(Ocs[i] - Os[k])))
                else:
                    r1 = Matrix(zcs[i].cross(Ocs[i] - Os[k]))               
                for t in xrange(3):
                    Jv[t, k] = r1[t, 0]
                    Jv[t + 3, k] = zcs[i][t, 0]
            if self.simplifying:
                Jvs.append(trigsimp(Jv))
            else:
                Jvs.append(Jv) 
        if self.simplifying:
            Jvs = [nsimplify(Jvs[i], [pi]) for i in xrange(len(Jvs))]  
            Ocs = [nsimplify(Ocs[i], [pi]) for i in xrange(len(Ocs))]             
        return Jvs, Ocs
    
    def transform(self, x, y, z, r, p, yaw):
        trans = Matrix([[1.0, 0.0, 0.0, x],
                        [0.0, 1.0, 0.0, y],
                        [0.0, 0.0, 1.0, z],
                        [0.0, 0.0, 0.0, 1.0]])
        roll = Matrix([[1.0, 0.0, 0.0, 0.0],
                       [0.0, cos(r), -sin(r), 0.0],
                       [0.0, sin(r), cos(r), 0.0],
                       [0.0, 0.0, 0.0, 1.0]])
        pitch = Matrix([[cos(p), 0.0, sin(p), 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [-sin(p), 0.0, cos(p), 0.0],
                        [0.0, 0.0, 0.0, 1.0]])
        yaw = Matrix([[cos(yaw), -sin(yaw), 0.0, 0.0],
                      [sin(yaw), cos(yaw), 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        
        res = roll * pitch * yaw * trans        
        return res
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Dynamic model generator.')
    parser.add_argument("-s", "--simplifying", 
                        help="Simplify the generated dynamic model", 
                        action="store_true")
    parser.add_argument("-b", "--buildcpp", 
                        help="Compile the c++ code after generating it", 
                        action="store_true")
    parser.add_argument("-m", "--model", help="Path to the robot model file")
    args = parser.parse_args()
    Test(args.model, args.simplifying, args.buildcpp)