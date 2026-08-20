cmake_minimum_required(VERSION 3.19)

project(exampleCartesianControlClient LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP QUIET 3.12 COMPONENTS os dev)

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS os dev)
    endif()
endif()

if(NOT TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

add_executable(exampleCartesianControlClient exampleCartesianControlClient.cpp)

target_link_libraries(exampleCartesianControlClient YARP::YARP_os
                                                    YARP::YARP_init
                                                    YARP::YARP_dev
                                                    ROBOTICSLAB::KinematicsDynamicsInterfaces)
