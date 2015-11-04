cmake_minimum_required (VERSION 2.6)

project (ompl_test)
set(CMAKE_BUILD_TYPE DEBUG)
#set(CMAKE_CXX_FLAGS "-g -std=c++11 -pg")
set(CMAKE_CXX_FLAGS "-g -std=c++11")
#SET(CMAKE_EXE_LINKER_FLAGS "-pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "-pg")
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ompl_test_SOURCE_DIR})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ompl_test_SOURCE_DIR})

include_directories(/usr/include/eigen3 
                    /usr/inclue/boost 
                    /usr/include/python2.7
                    /usr/local/include/openrave-0.9)

# set the path to the library folder
link_directories(/usr/lib /usr/local/lib /opt/ros/indigo/lib)

# link the libraries to the executable
add_library(robot SHARED robot.cpp)

add_library(integrate SHARED integrate.cpp)

add_library(dynamic_planner SHARED ompl_control.cpp)

#add_executable(ompl_test 
#               ompl_control.cpp 
#               state_propagator.cpp
#               propagator.cpp
#               integrate.cpp 
#               goal.cpp 
#               viewer.cpp 
#               torque_damper.cpp 
#               control_space.cpp 
#               control_sampler.cpp)
#set_property(TARGET ompl_test PROPERTY CXX_STANDARD 11)
target_link_libraries (dynamic_planner                        
                       openrave0.9 
                       openrave0.9-core 
                       ompl
                       boost_system
                       boost_python
                       python2.7 
                       boost_thread)
target_link_libraries (robot
                       urdf
                       boost_system
                       boost_python
                       python2.7)
target_link_libraries (integrate
                       boost_system
                       boost_python
                       python2.7)
set_property(TARGET robot PROPERTY CXX_STANDARD 11)
                       
#file(COPY "libintegrate.so" DESTINATION "..")
