#ifndef VIEWER_TEST_HPP_
#define VIEWER_TEST_HPP_
#include <iostream>
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace shared {
    class ViewerTest {
        public:
            ViewerTest();

            void testView(OpenRAVE::EnvironmentBasePtr &penv);

            void SetViewer(OpenRAVE::EnvironmentBasePtr &penv, const std::string &viewername);

    };

}

#endif
