#include "viewer.hpp"

using std::cout;
using std::endl;

namespace shared {

ViewerTest::ViewerTest() 
{

}

void ViewerTest::testView(OpenRAVE::EnvironmentBasePtr &penv) {    
    std::string viewername = "qtcoin";

    boost::thread thviewer(boost::bind(&ViewerTest::SetViewer, this, penv, viewername));
    //thviewer.join();
}

void ViewerTest::SetViewer(OpenRAVE::EnvironmentBasePtr &penv, const std::string &viewername)
{
    OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(penv, viewername);
    BOOST_ASSERT(!!viewer);
    // attach it to the environment:
    penv->Add(viewer);
    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

}
