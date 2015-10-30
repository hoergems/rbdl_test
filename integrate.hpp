#ifndef INTEGRATE_HPP_
#define INTEGRATE_HPP_
#include <Eigen/Dense>
#include <list>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>


namespace pl = std::placeholders;

using namespace Eigen;

namespace shared {

    typedef std::vector<double> state_type;

    class Integrate {
    public:
    	Integrate();
    	
    	void do_integration(std::vector<double> &x, std::vector<double> &int_times) const;
    	
    	void ode(const state_type &x , state_type &dxdt , double t) const;
    	
    	void setup(std::vector<double> &thetas_star, 
    			   std::vector<double> &dot_thetas_star, 
    			   std::vector<double> &rhos_star) const;
    	
    	std::vector<double> getResult();
    	
    private:
    	
    	MatrixXd getA(const state_type &x) const;
    	MatrixXd getB(const state_type &x) const;
    	
    	VectorXd getf(const state_type &x) const;
    	
    	mutable std::vector<double> thetas_star_;
    	
    	mutable std::vector<double> dot_thetas_star_;
    	
    	mutable std::vector<double> rhos_star_;
    	
    	mutable std::vector<double> rho;
    	
    	mutable std::vector<double> result_;
    	
    	
    };

}

#endif