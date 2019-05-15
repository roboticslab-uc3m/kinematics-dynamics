cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Start a project.
project(exampleScrewTheoryTrajectory LANGUAGES CXX)

find_package(YARP 3.0 REQUIRED COMPONENTS OS dev)
find_package(COLOR_DEBUG REQUIRED)
find_package(orocos_kdl 1.4 QUIET)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(exampleScrewTheoryTrajectory exampleScrewTheoryTrajectory.cpp
                                            TrajectoryThread.hpp
                                            TrajectoryThread.cpp)

target_link_libraries(exampleScrewTheoryTrajectory YARP::YARP_OS
                                                   YARP::YARP_init
                                                   YARP::YARP_dev
                                                   ${orocos_kdl_LIBRARIES}
                                                   ROBOTICSLAB::KdlVectorConverterLib
                                                   ROBOTICSLAB::KinematicRepresentationLib
                                                   ROBOTICSLAB::TrajectoryLib
                                                   ROBOTICSLAB::ScrewTheoryLib
                                                   ROBOTICSLAB::ColorDebug)

include(GNUInstallDirs)

install(TARGETS exampleScrewTheoryTrajectory
        DESTINATION ${CMAKE_INSTALL_BINDIR})
