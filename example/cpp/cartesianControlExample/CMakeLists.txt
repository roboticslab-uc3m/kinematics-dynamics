cmake_minimum_required(VERSION 2.8.0)

# Start a project.
project(cartesianControlExample)

find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

# Set up our main executable.
add_executable(cartesianControlExample CartesianControlExample.cpp)

target_link_libraries(cartesianControlExample YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev)

include(GNUInstallDirs)

install(TARGETS cartesianControlExample
        DESTINATION ${CMAKE_INSTALL_BINDIR})
