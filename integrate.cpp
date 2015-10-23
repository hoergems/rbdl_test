
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

MatrixXd Integrate::getA() const{
	MatrixXd a1 = getA1();
	MatrixXd a2 = getA2();
	MatrixXd a3 = getA3();
	MatrixXd A = a1 + a2 + a3;	
	MatrixXd m = MatrixXd::Zero(thetas_star_.size(), thetas_star_.size());
	MatrixXd res(m.rows() + A.rows(), m.cols());
	res << m,
		   A;
	return res;
}

MatrixXd Integrate::getB() const{
	MatrixXd b1 = getB1();
	MatrixXd m = MatrixXd::Identity(thetas_star_.size(), thetas_star_.size());
	MatrixXd res(m.rows() + b1.rows(), m.cols());
	res << m,
		   b1;
	return res;
}

MatrixXd Integrate::getC() const{
	MatrixXd c1 = getC1();
	MatrixXd m = MatrixXd::Zero(thetas_star_.size(), thetas_star_.size());
	MatrixXd res(m.rows() + c1.rows(), m.cols());
	res << m,
		   c1;
	return res;
}

VectorXd Integrate::getf() const{
VectorXd m(4); 
m(0, 0) = dot_thetas_star_[0]; 
m(1, 0) = dot_thetas_star_[1]; 
m(2, 0) = 2.5*pow(dot_thetas_star_[0], 2)*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - dot_thetas_star_[1]*(5.0*dot_thetas_star_[0] + 2.5*dot_thetas_star_[1])*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - rhos_star_[0]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - rhos_star_[1]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)/(-3.125*cos(2.0*thetas_star_[1]) + 3.275); 
m(3, 0) = (15.7*pow(dot_thetas_star_[0], 2)*sin(1.0*thetas_star_[1]) + 6.25*pow(dot_thetas_star_[0], 2)*sin(2.0*thetas_star_[1]) + 6.4*dot_thetas_star_[0]*dot_thetas_star_[1]*sin(1.0*thetas_star_[1]) + 6.25*dot_thetas_star_[0]*dot_thetas_star_[1]*sin(2.0*thetas_star_[1]) + 3.2*pow(dot_thetas_star_[1], 2)*sin(1.0*thetas_star_[1]) + 3.125*pow(dot_thetas_star_[1], 2)*sin(2.0*thetas_star_[1]) + 2.5*rhos_star_[0]*cos(1.0*thetas_star_[1]) + 1.28*rhos_star_[0] - 5.0*rhos_star_[1]*cos(1.0*thetas_star_[1]) - 6.28*rhos_star_[1])/(3.125*cos(2.0*thetas_star_[1]) - 3.275); 
return m; 
	
}

MatrixXd Integrate::getA1() const{
MatrixXd m(2, 2); 
m(0, 0) = 0; 
m(0, 1) = -6.25*rhos_star_[0]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(2.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*pow(3.125*cos(2.0*thetas_star_[1]) - 3.275, 2)) + 6.4*rhos_star_[0]*sin(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 5.0*rhos_star_[0]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])/(pow(5.0*cos(1.0*thetas_star_[1]) + 6.28, 2)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) + 6.25*rhos_star_[1]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(2.0*thetas_star_[1])/pow(-3.125*cos(2.0*thetas_star_[1]) + 3.275, 2) + 2.5*rhos_star_[1]*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275); 
m(1, 0) = 0; 
m(1, 1) = (-2.5*rhos_star_[0]*sin(1.0*thetas_star_[1]) + 5.0*rhos_star_[1]*sin(1.0*thetas_star_[1]))/(3.125*cos(2.0*thetas_star_[1]) - 3.275) + 6.25*(2.5*rhos_star_[0]*cos(1.0*thetas_star_[1]) + 1.28*rhos_star_[0] - 5.0*rhos_star_[1]*cos(1.0*thetas_star_[1]) - 6.28*rhos_star_[1])*sin(2.0*thetas_star_[1])/pow(3.125*cos(2.0*thetas_star_[1]) - 3.275, 2); 
return m; 
	
}

MatrixXd Integrate::getA2() const{
MatrixXd m(2, 2); 
m(0, 0) = 0; 
m(0, 1) = dot_thetas_star_[0]*(2.5*dot_thetas_star_[0]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*cos(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - 15.625*dot_thetas_star_[0]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])*sin(2.0*thetas_star_[1])/pow(-3.125*cos(2.0*thetas_star_[1]) + 3.275, 2) - 6.25*dot_thetas_star_[0]*pow(sin(1.0*thetas_star_[1]), 2)/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - 2.5*dot_thetas_star_[1]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*cos(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 15.625*dot_thetas_star_[1]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])*sin(2.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*pow(3.125*cos(2.0*thetas_star_[1]) - 3.275, 2)) + 16.0*dot_thetas_star_[1]*pow(sin(1.0*thetas_star_[1]), 2)/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 12.5*dot_thetas_star_[1]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*pow(sin(1.0*thetas_star_[1]), 2)/(pow(5.0*cos(1.0*thetas_star_[1]) + 6.28, 2)*(3.125*cos(2.0*thetas_star_[1]) - 3.275))) - 2.5*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*cos(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 15.625*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])*sin(2.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*pow(3.125*cos(2.0*thetas_star_[1]) - 3.275, 2)) + 16.0*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*pow(sin(1.0*thetas_star_[1]), 2)/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 12.5*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*pow(sin(1.0*thetas_star_[1]), 2)/(pow(5.0*cos(1.0*thetas_star_[1]) + 6.28, 2)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)); 
m(1, 0) = 0; 
m(1, 1) = dot_thetas_star_[0]*(-12.5*dot_thetas_star_[0]*sin(1.0*thetas_star_[1]) - 6.25*dot_thetas_star_[1]*sin(1.0*thetas_star_[1]))*sin(1.0*thetas_star_[1])/(3.125*cos(2.0*thetas_star_[1]) - 3.275) + 1.0*dot_thetas_star_[0]*(12.5*dot_thetas_star_[0]*cos(1.0*thetas_star_[1]) + 15.7*dot_thetas_star_[0] + 6.25*dot_thetas_star_[1]*cos(1.0*thetas_star_[1]) + 3.2*dot_thetas_star_[1])*cos(1.0*thetas_star_[1])/(3.125*cos(2.0*thetas_star_[1]) - 3.275) + 6.25*dot_thetas_star_[0]*(12.5*dot_thetas_star_[0]*cos(1.0*thetas_star_[1]) + 15.7*dot_thetas_star_[0] + 6.25*dot_thetas_star_[1]*cos(1.0*thetas_star_[1]) + 3.2*dot_thetas_star_[1])*sin(1.0*thetas_star_[1])*sin(2.0*thetas_star_[1])/pow(3.125*cos(2.0*thetas_star_[1]) - 3.275, 2) - 2.5*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*cos(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) + 15.625*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])*sin(2.0*thetas_star_[1])/pow(-3.125*cos(2.0*thetas_star_[1]) + 3.275, 2) + 6.25*dot_thetas_star_[1]*(dot_thetas_star_[0] + dot_thetas_star_[1])*pow(sin(1.0*thetas_star_[1]), 2)/(-3.125*cos(2.0*thetas_star_[1]) + 3.275); 
return m; 
	
}

MatrixXd Integrate::getA3() const{
MatrixXd m(2, 2); 
m(0, 0) = 0; 
m(0, 1) = 0; 
m(1, 0) = 0; 
m(1, 1) = 0; 
return m; 
	
}

MatrixXd Integrate::getB1() const{
MatrixXd m(2, 2); 
m(0, 0) = 5.0*dot_thetas_star_[0]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - 5.0*dot_thetas_star_[1]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)); 
m(0, 1) = -2.5*dot_thetas_star_[0]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 2.5*dot_thetas_star_[1]*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)) - 2.5*(dot_thetas_star_[0] + dot_thetas_star_[1])*(6.4*cos(1.0*thetas_star_[1]) + 8.0384)*sin(1.0*thetas_star_[1])/((5.0*cos(1.0*thetas_star_[1]) + 6.28)*(3.125*cos(2.0*thetas_star_[1]) - 3.275)); 
m(1, 0) = -5.0*dot_thetas_star_[0]*(5.0*cos(1.0*thetas_star_[1]) + 6.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - 5.0*dot_thetas_star_[1]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275); 
m(1, 1) = -2.5*dot_thetas_star_[0]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - 2.5*dot_thetas_star_[1]*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275) - 2.5*(dot_thetas_star_[0] + dot_thetas_star_[1])*(2.5*cos(1.0*thetas_star_[1]) + 1.28)*sin(1.0*thetas_star_[1])/(-3.125*cos(2.0*thetas_star_[1]) + 3.275); 
return m; 
	
}

MatrixXd Integrate::getC1() const {
MatrixXd m(2, 2); 
m(0, 0) = -(6.4*cos(1.0*dot_thetas_star_[1]) + 8.0384)/((5.0*cos(1.0*dot_thetas_star_[1]) + 6.28)*(3.125*cos(2.0*dot_thetas_star_[1]) - 3.275)); 
m(0, 1) = -(2.5*cos(1.0*dot_thetas_star_[1]) + 1.28)/(-3.125*cos(2.0*dot_thetas_star_[1]) + 3.275); 
m(1, 0) = -(2.5*cos(1.0*dot_thetas_star_[1]) + 1.28)/(-3.125*cos(2.0*dot_thetas_star_[1]) + 3.275); 
m(1, 1) = (5.0*cos(1.0*dot_thetas_star_[1]) + 6.28)/(-3.125*cos(2.0*dot_thetas_star_[1]) + 3.275); 
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
	for (size_t i = 0; i < x.size() / 2; i++) {
		q(i) = x[i];
		q_dot(i) = x[i + x.size() / 2];
		r(i) = rhos_star_[i];
		
		q_star(i) = thetas_star_[i];
		q_dot_star(i) = dot_thetas_star_[i];
		r_star(i) = rhos_star_[i];
	}
	
	MatrixXd A = getA();	
	MatrixXd B = getB();
	MatrixXd C = getC();
	MatrixXd f = getf();
	res = f + A * (q - q_star) + B * (q_dot - q_dot_star) + C * (r - r_star);
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