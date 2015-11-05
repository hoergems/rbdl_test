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

void Integrate::do_integration(std::vector<double> &x, 
		                       std::vector<double> &control,
		                       std::vector<double> &int_times) const {
	double t0 = int_times[0];
	double te = int_times[1];
	double step_size = int_times[2];
	
	rho.clear();
	for (size_t i = 0; i < control.size(); i++) {
		rho.push_back(control[i]);
	}
	
	closest_steady_state_ = getClosestSteadyState(x);
	for (size_t i = 0; i < closest_steady_state_.second.size(); i++) {
		if (closest_steady_state_.second[i] == -1) {
			closest_steady_state_.second[i] = x[i];
		}		
	}
	ab_functions_ = getClosestSteadyStateFunctions(closest_steady_state_.first);
	
	size_t k = integrate_const(runge_kutta4<state_type>() ,
		                       std::bind(&Integrate::ode , this , pl::_1 , pl::_2 , pl::_3),
		                       x , t0 , te , step_size);
	for (size_t i = 0; i < x.size(); i++) {
		result_.push_back(x[i]);
	}
}

void Integrate::setupSteadyStates() const {
std::vector<double> steady_state_0({-1, -1, 0.0, 0.0}); 
steady_states_.push_back(steady_state_0); 
a_map_.insert(std::make_pair(0, &Integrate::getA0)); 
b_map_.insert(std::make_pair(0, &Integrate::getB0)); 
	
}

std::pair<Integrate::AB_funct, Integrate::AB_funct> Integrate::getClosestSteadyStateFunctions(int &idx) const {
	return std::make_pair(a_map_.find(idx)->second, b_map_.find(idx)->second);
}

std::pair<int, std::vector<double>> Integrate::getClosestSteadyState(const state_type &x) const {
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
	
	return std::make_pair(min_idx, steady_states_[min_idx]);
}

void Integrate::ode(const state_type &x , state_type &dxdt , double t) const {
	VectorXd res(x.size());
	
	VectorXd state(x.size());
	VectorXd control_state(x.size() / 2);	
	for (size_t i = 0; i < x.size() / 2; i++) {
		state[i] = x[i] - closest_steady_state_.second[i];
		state[i + x.size() / 2] = x[i + x.size() / 2] - closest_steady_state_.second[i + x.size() / 2];
		control_state[i] = rho[i];
	}
	
	auto A = ab_functions_.first;
	auto B = ab_functions_.second;
	res = (this->*A)(closest_steady_state_.second) * state + (this->*B)(closest_steady_state_.second) * control_state;	
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
m(2, 2) = 17500.0/(15625.0*pow(cos(x[1]), 2) - 22400.0);
m(2, 3) = -(31250.0*cos(x[1]) + 17500.0)/(15625.0*pow(cos(x[1]), 2) - 22400.0); 
m(3, 0) = 0; 
m(3, 1) = 0; 
m(3, 2) = 50*(-25.0*cos(x[1]) - 14.0)/(625*pow(cos(x[1]), 2) - 896); 
m(3, 3) = (1.0L/2.0L)*(5000.0*cos(x[1]) + 7800.0)/(625*pow(cos(x[1]), 2) - 896); 
return m; 

} 
MatrixXd Integrate::getB0(const state_type &x) const{ 
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = -700/(625*pow(cos(x[1]), 2) - 896); 
m(2, 1) = 50*(25*cos(x[1]) + 14)/(625*pow(cos(x[1]), 2) - 896); 
m(3, 0) = 50*(25*cos(x[1]) + 14)/(625*pow(cos(x[1]), 2) - 896); 
m(3, 1) = -(2500*cos(x[1]) + 3900)/(625*pow(cos(x[1]), 2) - 896); 
return m; 

} 
 
}