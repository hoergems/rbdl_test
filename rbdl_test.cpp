#include "rbdl_test.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


using std::cout;
using std::endl;

namespace shared {

RbdlTest::RbdlTest():
    model_(new Model){    
}

bool RbdlTest::init(const std::string &model_file) {
    const char *c = model_file.c_str();    
    if (!Addons::URDFReadFromFile (c, model_, false)) {
		std::cerr << "Error loading model " << model_file << std::endl;
		abort();
    }
    cout << "loaded model with dof " << model_->dof_count << endl;
	
    for (std::map<std::string, unsigned int>::iterator it=model_->mBodyNameMap.begin(); it!=model_->mBodyNameMap.end(); ++it) {
        std::cout << it->first << " => " << it->second << '\n';
    }
}

void RbdlTest::calcDamping(std::vector<double> &pos,
                           std::vector<double> &vel,
                           std::vector<double> &oldVel,
                           double &delta_t,
                           std::vector<double> &torques){

    std::vector<double> accelerations;
    VectorNd Q = VectorNd::Zero(model_->dof_count);
    VectorNd QDot = VectorNd::Zero(model_->dof_count);
    VectorNd QDDot = VectorNd::Zero(model_->dof_count);
    VectorNd Tau = VectorNd::Zero(model_->dof_count);
    for (size_t i = 0; i < vel.size(); i++) {
        Q[i] = pos[i];
        QDot[i] = vel[i];
        QDDot[i] = (vel[i] - oldVel[i]) / delta_t;        
    }

    cout << "Q " << Q << endl;
    cout << "QDot " << QDot << endl;
    cout << "QDDot " << QDDot << endl;
    InverseDynamics(*model_, Q, QDot, QDDot, Tau);
    cout << "TAU " << Tau;
    for (size_t i = 0; i < pos.size(); i++) {
        torques.push_back(-Tau[i] * 0.1);
    }
}

int RbdlTest::test() {
    Model* model = new Model();
    if (!Addons::URDFReadFromFile ("./model2.urdf", model, false)) {
		std::cerr << "Error loading model ./model2.urdf" << std::endl;
		abort();
	}
	cout << "loaded model with dof " << model->dof_count << endl;
	
	for (std::map<std::string, unsigned int>::iterator it=model->mBodyNameMap.begin(); it!=model->mBodyNameMap.end(); ++it) {
        std::cout << it->first << " => " << it->second << '\n';
    }
	
	VectorNd Q = VectorNd::Zero (model->dof_count);
	VectorNd QDot = VectorNd::Zero (model->dof_count);
	VectorNd Tau = VectorNd::Zero (model->dof_count);
	Tau[1] = -0.365336;
	Tau[2] = -0.17102;
	VectorNd QDDot = VectorNd::Zero (model->dof_count);
	
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	std::cout << Q.transpose() << std::endl;
	std::cout << QDDot << std::endl;
	
	Q = VectorNd::Zero(model->dof_count);
	QDot = VectorNd::Zero(model->dof_count);
	QDDot = VectorNd::Zero(model->dof_count);
	Tau = VectorNd::Zero(model->dof_count);
	
	InverseDynamics(*model, Q, QDot, QDDot, Tau);
	
	cout << "torques: " << Tau << endl;

	delete model;

 	return 0;
}

}


