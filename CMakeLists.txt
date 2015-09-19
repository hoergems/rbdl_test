cmake_minimum_required (VERSION 2.6)

project (ompl_test)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS "-g -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11")

include_directories(/usr/local/include/rbdl
                    /usr/include/eigen3 
                    /usr/inclue/boost 
                    /usr/local/include/openrave-0.9)

# set the path to the library folder
link_directories(/usr/lib /usr/local/lib)

# link the libraries to the executable


add_executable(ompl_test ompl_control.cpp state_propagator.cpp goal.cpp)
set_property(TARGET ompl_test PROPERTY CXX_STANDARD 11)
target_link_libraries (ompl_test 
                       rbdl
                       rbdl_urdfreader 
                       openrave0.9 
                       openrave0.9-core 
                       ompl
                       boost_system 
                       boost_thread)
