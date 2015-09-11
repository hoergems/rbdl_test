#include "rbdl_test.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


using std::cout;
using std::endl;

namespace rbdl_test {

RbdlTest::RbdlTest() {    
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
	Tau[0] = 0.005;
	//Tau[2] = 0.005;
	VectorNd QDDot = VectorNd::Zero (model->dof_count);
	
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	std::cout << Q.transpose() << std::endl;
	std::cout << QDDot.transpose() << std::endl;
	
	/**VectorNd Q = VectorNd::Zero(model->dof_count);
	VectorNd QDot = VectorNd::Zero(model->dof_count);
	VectorNd QDDot = VectorNd::Zero(model->dof_count);
	VectorNd Tau = VectorNd::Zero(model->dof_count);
	
	InverseDynamics(*model, Q, QDot, QDDot, Tau);
	
	cout << "torques: " << Tau << endl;*/

	delete model;

 	return 0;
}

}

int main(int argc, char** argv) {
    rbdl_test::RbdlTest rbdl;
    rbdl.test();
    return 0;
}
