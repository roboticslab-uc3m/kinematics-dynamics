cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(exampleScrewTheoryTrajectory LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::KdlVectorConverterLib
        OR NOT TARGET ROBOTICSLAB::KinematicRepresentationLib
        OR NOT TARGET ROBOTICSLAB::TrajectoryLib
        OR NOT TARGET ROBOTICSLAB::ScrewTheoryLib)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

find_package(orocos_kdl 1.4 QUIET)

add_executable(exampleScrewTheoryTrajectory exampleScrewTheoryTrajectory.cpp
                                            TrajectoryThread.hpp
                                            TrajectoryThread.cpp)

target_link_libraries(exampleScrewTheoryTrajectory YARP::YARP_os
                                                   YARP::YARP_init
                                                   YARP::YARP_dev
                                                   ${orocos_kdl_LIBRARIES}
                                                   ROBOTICSLAB::KdlVectorConverterLib
                                                   ROBOTICSLAB::KinematicRepresentationLib
                                                   ROBOTICSLAB::TrajectoryLib
                                                   ROBOTICSLAB::ScrewTheoryLib)
