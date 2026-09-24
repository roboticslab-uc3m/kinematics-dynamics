cmake_minimum_required(VERSION 3.19)

project(exampleScrewTheoryTrajectory LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP QUIET 3.12 COMPONENTS os dev)

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS os dev)
    endif()
endif()

if(NOT TARGET ROBOTICSLAB::ScrewTheorySolvers)
    find_package(roboticslab-screw-theory-solvers REQUIRED)
endif()

find_package(orocos_kdl 1.4 QUIET)

add_executable(exampleScrewTheoryTrajectory exampleScrewTheoryTrajectory.cpp
                                            TrajectoryThread.hpp
                                            TrajectoryThread.cpp)

target_link_libraries(exampleScrewTheoryTrajectory YARP::YARP_os
                                                   YARP::YARP_init
                                                   YARP::YARP_dev
                                                   ${orocos_kdl_LIBRARIES}
                                                   ROBOTICSLAB::ScrewTheorySolvers)

target_include_directories(exampleScrewTheoryTrajectory PRIVATE ${orocos_kdl_INCLUDE_DIRS})
