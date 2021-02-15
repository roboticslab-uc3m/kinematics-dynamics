cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleCartesianControlClient LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

find_package(COLOR_DEBUG REQUIRED)

add_executable(exampleCartesianControlClient exampleCartesianControlClient.cpp)

target_link_libraries(exampleCartesianControlClient YARP::YARP_os
                                                    YARP::YARP_init
                                                    YARP::YARP_dev
                                                    ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                    ROBOTICSLAB::ColorDebug)
