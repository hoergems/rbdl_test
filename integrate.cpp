
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
	for (size_t i = 0; i < thetas_star.size(); i++) {
		thetas_star_.push_back(thetas_star[i]);
		dot_thetas_star_.push_back(dot_thetas_star[i]);
		rhos_star_.push_back(rhos_star[i]);
		rho.push_back(rhos_star[i]);
	}	
}

MatrixXd Integrate::getA(const state_type &x) const{
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

MatrixXd Integrate::getB(const state_type &x) const{
MatrixXd m(4, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
m(2, 0) = 1.0/(1.0*cos(1.0*x[1]) + 1.28) + pow(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03, 2)/((1.0*cos(1.0*x[1]) + 1.28)*(-0.125*cos(2.0*x[1]) + 0.155)); 
m(2, 1) = -(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03)/(-0.125*cos(2.0*x[1]) + 0.155); 
m(3, 0) = -(-0.5*(-1.0*sin(1.0*x[0]) - 0.5*sin(1.0*x[0] + 1.0*x[1]))*sin(1.0*x[0] + 1.0*x[1]) + 0.5*(1.0*cos(1.0*x[0]) + 0.5*cos(1.0*x[0] + 1.0*x[1]))*cos(1.0*x[0] + 1.0*x[1]) + 0.03)/(-0.125*cos(2.0*x[1]) + 0.155); 
m(3, 1) = (1.0*cos(1.0*x[1]) + 1.28)/(-0.125*cos(2.0*x[1]) + 0.155); 
return m; 
	
}


VectorXd Integrate::getf(const state_type &x) const{
VectorXd m(4); 
m(0, 0) = 0.0; 
m(1, 0) = 0.0; 
m(2, 0) = 0; 
m(3, 0) = 0; 
return m; 
	
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
		//control_state[i] = rho[i] - rhos_star_[i];
		control_state[i] = rho[i];
	}	
	/**for (size_t i = 0; i < x.size() / 2; i++) {
		q(i) = x[i];
		q_dot(i) = x[i + x.size() / 2];
		r(i) = rhos_star_[i];
		
		q_star(i) = thetas_star_[i];
		q_dot_star(i) = dot_thetas_star_[i];
		r_star(i) = rhos_star_[i];
	}*/
	
	MatrixXd A = getA(x);	
	MatrixXd B = getB(x);	
    //MatrixXd f = getf(x);
	cout << "A " << A << endl << endl;
	cout << "B " << B << endl << endl;	
	//cout << "f " << f << endl << endl;
	cout << "state " << state << endl << endl;
	cout << "control_state " << control_state << endl << endl;
	cout << "A * state " << A * state << endl << endl;
	cout << "B * control_state " << B * control_state << endl << endl;
	
	res = A * state + B * control_state;
	cout << "res " << res << endl << endl;
	//sleep(10);
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

}