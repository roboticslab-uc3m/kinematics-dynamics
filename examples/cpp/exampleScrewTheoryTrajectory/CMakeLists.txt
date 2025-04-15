cmake_minimum_required(VERSION 3.16)

project(exampleScrewTheoryTrajectory LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.10 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::ScrewTheoryLib)
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
                                                   ROBOTICSLAB::ScrewTheoryLib)

target_include_directories(exampleScrewTheoryTrajectory PRIVATE ${orocos_kdl_INCLUDE_DIRS})
