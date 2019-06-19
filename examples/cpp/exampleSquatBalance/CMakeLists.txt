cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Start a project.
project(exampleSquatBalance LANGUAGES CXX)

find_package(YARP 3.1 REQUIRED COMPONENTS OS dev)
find_package(COLOR_DEBUG REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(exampleSquatBalance exampleSquatBalance.cpp
                                   TrajectoryThread.hpp
                                   TrajectoryThread.cpp)

target_link_libraries(exampleSquatBalance YARP::YARP_OS
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          ROBOTICSLAB::TrajectoryLib
                                          ROBOTICSLAB::ColorDebug)

include(GNUInstallDirs)

install(TARGETS exampleSquatBalance
        DESTINATION ${CMAKE_INSTALL_BINDIR})
