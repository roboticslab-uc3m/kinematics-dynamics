cmake_minimum_required(VERSION 3.16)

project(exampleCartesianControlClient LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.10 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

add_executable(exampleCartesianControlClient exampleCartesianControlClient.cpp)

target_link_libraries(exampleCartesianControlClient YARP::YARP_os
                                                    YARP::YARP_init
                                                    YARP::YARP_dev
                                                    ROBOTICSLAB::KinematicsDynamicsInterfaces)
