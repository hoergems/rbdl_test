#include "integrate.hpp"

using namespace boost::numeric::odeint;
using std::endl;
using std::cout;

namespace shared{

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

Integrate::Integrate() {
	setupSteadyStates();
}

std::vector<double> Integrate::getResult() {
	return result_;
}

void Integrate::do_integration(std::vector<double> &x, std::vector<double> &int_times) const {
	double t0 = int_times[0];
	double te = int_times[1];
	double step_size = int_times[2];	
	size_t k = integrate_const(runge_kutta4<state_type>() ,
		                       std::bind(&Integrate::ode , this , pl::_1 , pl::_2 , pl::_3),
		                       x , t0 , te , step_size);
	for (size_t i = 0; i < x.size(); i++) {
		result_.push_back(x[i]);
	}
}

void Integrate::setup(std::vector<double> &thetas_star, 
		              std::vector<double> &dot_thetas_star, 
		              std::vector<double> &rhos_star) const {	
	thetas_star_.clear();
	dot_thetas_star_.clear();
	rhos_star_.clear();
	rho.clear();
	state_type state_temp;
	for (size_t i = 0; i < thetas_star.size(); i++) {
		thetas_star_.push_back(thetas_star[i]);
		dot_thetas_star_.push_back(dot_thetas_star[i]);
		rhos_star_.push_back(rhos_star[i]);
		rho.push_back(rhos_star[i]);
		
		state_temp.push_back(thetas_star[i]);
	}
	
	
	for (size_t i = 0; i < dot_thetas_star.size(); i++) {
		state_temp.push_back(dot_thetas_star[i]);
	}
	
	const state_type state = state_temp;
	
	ab_functions_ = getClosestSteadyStateFunctions(state);
	
}

void Integrate::setupSteadyStates() const {
std::vector<double> steady_state_0({-1, -1, 0.0, 0.0}); 
steady_states_.push_back(steady_state_0); 
a_map_.insert(std::make_pair(0, &Integrate::getA0)); 
b_map_.insert(std::make_pair(0, &Integrate::getB0)); 
	
}

std::pair<Integrate::AB_funct, Integrate::AB_funct> Integrate::getClosestSteadyStateFunctions(const state_type &x) const {
	int min_idx = 0;
	double dist = 0.0;
	double min_dist = 10000000.0;
	double steady_state_val = 0.0;
	for (size_t i = 0; i < steady_states_.size(); i++) {
		dist = 0.0;		
		for (size_t j = 0; j < steady_states_[i].size(); j++) {
			if (steady_states_[i][j] == -1) {
				steady_state_val = x[j];
			}
			else {			    
			    steady_state_val = steady_states_[i][j];
			}
			
			dist += std::pow(x[j] - steady_state_val, 2);
		}
		
		dist = std::sqrt(dist);
		if (dist < min_dist) {
			min_dist = dist;
			min_idx = i;
		}		
	}
	
	return std::make_pair(a_map_.find(min_idx)->second, b_map_.find(min_idx)->second);
}



void Integrate::ode(const state_type &x , state_type &dxdt , double t) const {
	VectorXd res(x.size());
	VectorXd q(x.size() / 2);
	VectorXd q_dot(x.size() / 2);
	VectorXd r(x.size() / 2);
	
	VectorXd q_star(x.size() / 2);
	VectorXd q_dot_star(x.size() / 2);
	VectorXd r_star(x.size() / 2);
	
	VectorXd state(x.size());
	VectorXd control_state(x.size() / 2);
	
	for (size_t i = 0; i < x.size() / 2; i++) {
		state[i] = x[i] - thetas_star_[i];
		state[i + x.size() / 2] = x[i + x.size() / 2] - dot_thetas_star_[i];
		control_state[i] = rho[i];
	}
	
	auto A = ab_functions_.first;
	auto B = ab_functions_.second;
	res = (this->*A)(x) * state + (this->*B)(x) * control_state;
 	dxdt.clear();
	for (size_t i = 0; i < x.size(); i++) {		
		dxdt.push_back(res(i));
	}	
} 

BOOST_PYTHON_MODULE(libintegrate) {
    using namespace boost::python;
    
    class_<std::vector<double> > ("v_double")
             .def(vector_indexing_suite<std::vector<double> >());
    
    class_<Integrate>("Integrate", init<>())
                        .def("doIntegration", &Integrate::do_integration) 
                        .def("setup", &Integrate::setup)
                        .def("getResult", &Integrate::getResult)
    ;
}

MatrixXd Integrate::getA0(const state_type &x) const{ 
MatrixXd m(4, 4); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(0, 2) = 1; 
m(0, 3) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(1, 2) = 0; 
m(1, 3) = 1; 
m(2, 0) = 0; 
m(2, 1) = 0; 
m(2, 2) = 0; 
m(2, 3) = 0; 
m(3, 0) = 0; 
m(3, 1) = 0; 
m(3, 2) = 0; 
m(3, 3) = 0; 
return m; 

} 
MatrixXd Integrate::getB0(const state_type &x) const{ 
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = (pow(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03, 2)/((pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03)*(-pow(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03, 2)/(pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03) + 0.25*pow(sin(1.0*x[0] + 1.0*x[1]), 2) + 0.25*pow(cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03)) + 1)/(pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03); 
m(2, 1) = -(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03)/((pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03)*(-pow(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03, 2)/(pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03) + 0.25*pow(sin(1.0*x[0] + 1.0*x[1]), 2) + 0.25*pow(cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03)); 
m(3, 0) = -(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03)/((pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03)*(-pow(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03, 2)/(pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03) + 0.25*pow(sin(1.0*x[0] + 1.0*x[1]), 2) + 0.25*pow(cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03)); 
m(3, 1) = 1.0/(-pow(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03, 2)/(pow(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]), 2) + pow(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03) + 0.25*pow(sin(1.0*x[0] + 1.0*x[1]), 2) + 0.25*pow(cos(1.0*x[0] + 1.0*x[1]), 2) + 0.03); 
return m; 

} 
 
}