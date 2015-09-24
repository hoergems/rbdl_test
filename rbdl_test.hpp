#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/addons/luamodel/luamodel.h>

namespace shared {

class RbdlTest {
    public: 
        RbdlTest();
        
        int test();

        bool init(const std::string &model_file);

        void calcDamping(std::vector<double> &pos,
                         std::vector<double> &vel,
                         std::vector<double> &oldVel,
                         double &delta_t,
                         std::vector<double> &torques);

    private:
        RigidBodyDynamics::Model* model_;
};
}
