cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# Start a project.
project(exampleCartesianControlClient)

find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(exampleCartesianControlClient exampleCartesianControlClient.cpp)

target_link_libraries(exampleCartesianControlClient YARP::YARP_OS
                                                    YARP::YARP_init
                                                    YARP::YARP_dev
                                                    ROBOTICSLAB::KinematicsDynamicsInterfaces)

include(GNUInstallDirs)

install(TARGETS exampleCartesianControlClient
        DESTINATION ${CMAKE_INSTALL_BINDIR})
