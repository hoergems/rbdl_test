from librobot import *
from sympy import *
import numpy as np
import time
import os
import sys
import argparse
from sympy.printing import print_ccode
from sympy.abc import x

from scipy.integrate import ode, odeint
from gi.overrides.keysyms import R10

class Test:
    def __init__(self, model, simplifying, buildcpp):
        self.simplifying = simplifying
        self.parse_urdf(model)
        g = 9.81        
        """
        Get the Jacobians of the links expressed in the robot's base frame
        """
        print "Calculating Jacobian matrices"
        Jvs, Ocs = self.get_link_jacobians(self.joint_origins, self.inertial_poses, self.joint_axis, self.q)
        M_is = self.construct_link_inertia_matrices(self.link_masses, self.Is)        
        print "Calculating inertia matrix"
        M = self.calc_inertia_matrix(Jvs, M_is)        
        if self.simplifying:
            M = trigsimp(M)                  
        print "Calculating coriolis matrix"
        C = self.calc_coriolis_matrix(self.q, self.qdot, M)
        if self.simplifying:
            C = trigsimp(C)
        print "Calculating normal forces" 
        N = self.calc_generalized_forces(self.q,
                                         self.qdot, 
                                         Ocs, 
                                         self.link_masses, 
                                         g) 
        if self.simplifying:      
            N = trigsimp(N)
        print "Get dynamic model"        
        f, M_inv = self.get_dynamic_model(M, C, N, self.q, self.qdot, self.rho)           
        print "Build taylor approximation" 
        #(fot, f1, A, B, C) = self.taylor_approximation(f, self.q, self.qdot, self.qstar, self.qdotstar, self.rho, self.rhostar)
        #self.lin(fot, f1, A, B, C)       
        (f2, A1, A2, A3, B1, C1) = self.partial_derivatives(f,
                                                           M_inv,
                                                           C,
                                                           N,
                                                           self.q,
                                                           self.qdot,
                                                           self.rho,
                                                           self.qstar, 
                                                           self.qdotstar, 
                                                           self.rhostar)
        print "Finding steady states..."
        steady_states = self.get_steady_states(f2)        
        
        (f, A, B) = self.substitude_steady_states(f2, A1, A2, A3, B1, C1, steady_states)
        print "Generating c++ code..."
        self.gen_cpp_code2(A, "A")
        self.gen_cpp_code2(B, "B")        
        self.gen_cpp_code2(f, "f")
        if buildcpp:
            print "Build c++ code..."
            cmd = "cd build && cmake .. && make -j8"           
            os.system(cmd)
        print "Done"    
        
    def get_steady_states(self, f):        
        for i in xrange(len(self.rho)):
            f = f.subs(self.rho[i], 0)
        print "simplifying fs..."
        '''for i in xrange(len(f)):
            f[i, 0] = trigsimp(f[i, 0])'''
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
        return steady_states[0]
        
    def substitude_steady_states(self, f, A1, A2, A3, B1, C1, steady_states):                             
        for i in xrange(len(self.rho)):            
            f = f.subs(self.rho[i], 0)
            A1 = A1.subs(self.rho[i], 0)
            A2 = A2.subs(self.rho[i], 0)
            A3 = A3.subs(self.rho[i], 0)
            B1 = B1.subs(self.rho[i], 0)
            C1 = C1.subs(self.rho[i], 0)            
        for i in xrange(len(steady_states.keys())):
            
            f = f.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
            A1 = A1.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
            A2 = A2.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
            A3 = A3.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
            B1 = B1.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])
            C1 = C1.subs(steady_states.keys()[i], steady_states[steady_states.keys()[i]])              
        A = Matrix([[0, 0],
                    [0, 0]])
        A_low = A1 + A2 + A3        
        A = A.col_join(A_low)
            
        B = Matrix([[1, 0],
                    [0, 1]])
        B = B.col_join(B1)
        A = A.row_join(B)
        
        B = Matrix([[0, 0],
                    [0, 0]])
        B = B.col_join(C1)   
        
        return f, A, B
    
        
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
        self.joint_origins = [Matrix([[joint_origins[i][j]] for j in xrange(len(joint_origins[i]))]) for i in xrange(len(joint_origins))]
        
        joint_axis = v2_double()
        robot.getJointAxis(joint_names, joint_axis)
        self.joint_axis = [Matrix([[joint_axis[i][j]] for j in xrange(len(joint_axis[i]))]) for i in xrange(len(joint_axis))]
        
        self.q = []
        self.qdot = []
        self.qstar = []
        self.qdotstar = []
        self.rho = []
        self.rhostar = []
        for i in xrange(len(self.joint_names)):
            #if self.joint_types[i] == "revolute":            
            
            symb_string_q = "x[" + str(i) + "]"
            symb_string_q_dot = "x[" + str(i + len(self.joint_names) - 1) + "]"
            symb_string_q_star = "thetas_star_[" + str(i) + "]"
            symb_string_q_dot_star = "dot_thetas_star_[" + str(i) + "]"
            symb_string_r = "rho[" + str(i) + "]"
            symb_string_r_star = "rhos_star_[" + str(i) + "]"           
            
            
            self.q.append(symbols(symb_string_q))
            self.qdot.append(symbols(symb_string_q_dot))
            self.rho.append(symbols(symb_string_r))
            self.qstar.append(symbols(symb_string_q_star))
            self.qdotstar.append(symbols(symb_string_q_dot_star))
            self.rhostar.append(symbols(symb_string_r_star))                  
            '''else: 
                sy = symbols('c')               
                self.q.append(sy)
                self.qdot.append(sy)
                self.qstar.append(sy)
                self.qdotstar.append(sy)
                self.rho.append(sy)
                self.rhostar.append(sy)'''             
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
     
    def gen_cpp_code2(self, Matr, name):
        lines = list(open("integrate.cpp", 'r'))
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
            if "Integrate::get" + name in lines[i]:                
                idx1 = i + 1               
                breaking = True
            elif "}" in lines[i]:
                idx2 = i - 1
                if breaking:
                    break        
        del lines[idx1:idx2]
        idx = -1
        for i in xrange(len(lines)):
            if "Integrate::get" + name in lines[i]:
                idx = i        
        lines[idx+1:idx+1] = temp_lines        
        os.remove("integrate.cpp")
        with open("integrate.cpp", 'a+') as f:
            for line in lines:
                f.write(line)
                            
            
                
    def gen_cpp_code(self, fot):
        lines = list(open("integrate.cpp", 'r'))
        idx1 = -1 
        idx2 = -1 
        temp_lines = []     
        for i in xrange(len(lines)):
            if "void Integrate::ode" in lines[i]:
                idx1 = i 
            elif "BOOST_PYTHON_MODULE(libintegrate)" in lines[i]:
                idx2 = i
        temp_lines.append("\n")
        temp_lines.append("void Integrate::ode(const state_type &x , state_type &dxdt , double t) const { \n")       
        temp_lines.append("std::vector<double> terms({")        
        for i in xrange(len(fot)):
            temp_lines.append(ccode(fot[i]))            
            if i != len(fot) - 1:
                temp_lines.append(", \n")               
        temp_lines.append("}); \n")       
        temp_lines.append("dxdt.clear(); \n")
        temp_lines.append("for(size_t i = 0; i < x.size(); i++) { dxdt.push_back(terms[i]); } \n")
        temp_lines.append("} \n")       
        
        del lines[idx1:idx2]
        idx = -1
        for i in xrange(len(lines)):
            if "BOOST_PYTHON_MODULE(libintegrate)" in lines[i]:
                idx = i
        lines[idx-1:idx-1] = temp_lines    
        '''if not idx1 == -1:
            lines[idx1] = cpp_string'''
            
        os.remove("integrate.cpp")
        with open("integrate.cpp", 'a+') as f:
            for line in lines:
                f.write(line)
        
        
    def get_dynamic_model(self, M, C, N, thetas, dot_thetas, rs): 
        print "Inverting inertia matrix"              
        t0 = time.time()
        #M_inv = M.inv("LU")
        M_inv = M.inv()
        
        print "Inverted inertia matrix. Simplifying..."        
        print "time to invert: " + str(time.time() - t0)        
        Thetas = Matrix([[thetas[i]] for i in xrange(len(thetas) - 1)])
        Dotthetas = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        Rs = Matrix([[rs[i]] for i in xrange(len(rs) - 1)])
        print "Constructing non-linear differential equation"
        m_upper = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        m_lower = 0
        if self.simplifying:
            m_lower = trigsimp(-M_inv * trigsimp(C * Dotthetas + N) + M_inv * Rs) 
            
        else:            
            #m_lower = -M_inv * (C * Dotthetas + N) + M_inv * Rs 
            m_lower = M_inv * (Rs - C * Dotthetas - N)
            #m_lower = (C * Dotthetas) #+ M_inv * Rs                   
        h = m_upper.col_join(m_lower)        
        return h, M_inv
    
    def partial_derivatives(self,
                            f, 
                            M_inv, 
                            C, 
                            N, 
                            thetas, 
                            dot_thetas, 
                            rs, 
                            thetas_star,
                            dot_thetas_star,
                            rs_star):        
        dot_q = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        r = Matrix([[rs[i]] for i in xrange(len(rs) - 1)])
        
        q_star = Matrix([[thetas_star[i]] for i in xrange(len(thetas_star) - 1)])
        dot_q_star = Matrix([[dot_thetas_star[i]] for i in xrange(len(dot_thetas_star) - 1)])
        r_star = Matrix([[rs_star[i]] for i in xrange(len(rs_star) - 1)])
        
        A1 = M_inv * r
        A2 = -M_inv * C * dot_q
        A3 = -M_inv * N 
        if self.simplifying:
            A1 = trigsimp(A1)
            A2 = trigsimp(A2)
            A3 = trigsimp(A3)                   
        A1 = A1.jacobian([thetas[i] for i in xrange(len(thetas) - 1)])
        A2 = A2.jacobian([thetas[i] for i in xrange(len(thetas) - 1)])
        A3 = A3.jacobian([thetas[i] for i in xrange(len(thetas) - 1)])
        
        B1 = -M_inv * C * dot_q
        B1 = B1.jacobian([dot_thetas[i] for i in xrange(len(dot_thetas) - 1)])       
        
        C1 = M_inv * r        
        C1 = C1.jacobian([rs[i] for i in xrange(len(rs) - 1)])        
        '''for i in xrange(len(thetas) - 1):
            A1 = A1.subs(thetas[i], thetas_star[i])
            A1 = A1.subs(dot_thetas[i], dot_thetas_star[i])
            A1 = A1.subs(rs[i], rs_star[i])
            
            A2 = A2.subs(thetas[i], thetas_star[i])
            A2 = A2.subs(dot_thetas[i], dot_thetas_star[i])
            A2 = A2.subs(rs[i], rs_star[i])
            
            A3 = A3.subs(thetas[i], thetas_star[i])
            A3 = A3.subs(dot_thetas[i], dot_thetas_star[i])
            A3 = A3.subs(rs[i], rs_star[i])
            
            B1 = B1.subs(thetas[i], thetas_star[i])
            B1 = B1.subs(dot_thetas[i], dot_thetas_star[i])
            B1 = B1.subs(rs[i], rs_star[i])
           
            C1 = C1.subs(thetas[i], thetas_star[i])
            C1 = C1.subs(dot_thetas[i], dot_thetas_star[i])
            C1 = C1.subs(rs[i], rs_star[i])
            
            f = f.subs(thetas[i], thetas_star[i])
            f = f.subs(dot_thetas[i], dot_thetas_star[i])
            f = f.subs(rs[i], rs_star[i])'''
        return (f, A1, A2, A3, B1, C1)
         
        
    def taylor_approximation(self, f, thetas, dot_thetas, thetas_star, dot_thetas_star, rs, rs_star):        
        print "Calculate partial derivatives..."
        A = f.jacobian([thetas[i] for i in xrange(len(thetas) - 1)])        
        B = f.jacobian([dot_thetas[i] for i in xrange(len(dot_thetas) - 1)])
        C = f.jacobian([rs[i] for i in xrange(len(rs) - 1)])
        for i in xrange(len(thetas) - 1):
            A = A.subs(thetas[i], thetas_star[i])
            A = A.subs(dot_thetas[i], dot_thetas_star[i])
            A = A.subs(rs[i], rs_star[i])
            
            B = B.subs(thetas[i], thetas_star[i])
            B = B.subs(dot_thetas[i], dot_thetas_star[i])
            B = B.subs(rs[i], rs_star[i])
            
            C = C.subs(thetas[i], dot_thetas[i])
            C = C.subs(dot_thetas[i], dot_thetas_star[i])
            C = C.subs(rs[i], rs_star[i])
            
            f = f.subs(thetas[i], thetas_star[i])
            f = f.subs(dot_thetas[i], dot_thetas_star[i])
            f = f.subs(rs[i], rs_star[i])
            
        if self.simplifying:
            print "Simplifying Jacobians..."         
            A = trigsimp(A)
            B = trigsimp(B)
            C = trigsimp(C)
        q = Matrix([[thetas[i]] for i in xrange(len(thetas) - 1)])        
        dot_q = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])
        r = Matrix([[rs[i]] for i in xrange(len(rs) - 1)])
        
        q_star = Matrix([[thetas_star[i]] for i in xrange(len(thetas_star) - 1)])
        dot_q_star = Matrix([[dot_thetas_star[i]] for i in xrange(len(dot_thetas_star) - 1)])
        r_star = Matrix([[rs_star[i]] for i in xrange(len(rs_star) - 1)])        
        
        #sleep
        "Construct Taylor approximation..."
        fot = f + A * (q - q_star) + B * (dot_q - dot_q_star) + C * (r - r_star)
        return fot, f, A, B, C
        
    def test_fot(self, f):
        q1, q2, qdot1, qdot2, qdotdot1, qdotdot2 = symbols("q1 q2 qdot1 qdot2 qdotdot1 qdotdot2")
        r1, r2 = symbols("r1 r2")       
        x1_1, x1_2, x2_1, x2_2, x3_1, x3_2 = symbols("x1_1 x1_2 x2_1 x2_2 x3_1 x3_2")
        
        self.q = [q1, q2]
        self.qdot = [qdot1, qdot2]
        self.r = [r1, r2]
        
        self.q_star = [x1_1, x1_2]
        self.qdot_star = [x2_1, x2_2]
        self.r_star = [x3_1, x3_2]
        self.initial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        self.fot = self.taylor_approximation(f, 
                                             q, 
                                             qdot, 
                                             q_star, 
                                             qdot_star, 
                                             r, 
                                             r_star)        
        q_star.extend(qdot_star)
        q_star.extend(r_star)
        print trigsimp(fot)
        for i in xrange(len(q_star)):
            fot = fot.subs(q_star[i], self.initial[0])
        
        
        #print fot.subs([(q, 0.0), (q_dot, 0.0), (r, 1.0)])
        t = np.linspace(0.0, 0.3, 2)
        
        t0 = time.time()
        eq = odeint(self.f, np.array(self.initial[:4]), t)
        print "calc took " + str(time.time() - t0) + " seconds"
        print "================="
        print eq
        
    def calc_generalized_forces(self, 
                                thetas, 
                                dot_thetas, 
                                Ocs, 
                                ms, 
                                g):
        V = 0.0               
        for i in xrange(len(Ocs)): 
            el = ms[i + 1] * g * Ocs[i][2]                                           
            V += el
        
        N = 0
        if self.simplifying:    
            N = Matrix([[trigsimp(diff(V, thetas[i]))] for i in xrange(len(thetas) - 1)]) 
        else:
            N = Matrix([[diff(V, thetas[i])] for i in xrange(len(thetas) - 1)])        
        return N        
        
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
        return M_is
        
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
        #print joint_origins[len(com_coordinates) - 1][5]
        '''print (joint_origins[0][3] + axis[0][0] * thetas[0],
               joint_origins[0][4] + axis[0][1] * thetas[0],
               joint_origins[0][5] + axis[0][2] * thetas[0])
        sleep'''
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
            Ocs.append(trigsimp(O))
            zcs.append(z)
            res = res * trans_matrices2[i]            
            col3 = res.col(2)
            col4 = res.col(3)            
            z = Matrix([col3[j] for j in xrange(3)])
            O = Matrix([col4[j] for j in xrange(3)])
            Os.append(trigsimp(O))
            zs.append(z)        
        #r1 = trigsimp(Matrix([zcs[0].cross(Ocs[1] - Os[1])]))
        Jvs = []
        for i in xrange(len(thetas) - 1):
            Jv = Matrix([[0.0 for m in xrange(len(thetas) - 1)] for n in xrange(6)])
            for k in xrange(i + 1):
                r1 = trigsimp(Matrix(zcs[i].cross(Ocs[i] - Os[k])))                
                for t in xrange(3):
                    Jv[t, k] = r1[t, 0]
                    Jv[t + 3, k] = zcs[i][t, 0]
            
            Jvs.append(trigsimp(Jv))                
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