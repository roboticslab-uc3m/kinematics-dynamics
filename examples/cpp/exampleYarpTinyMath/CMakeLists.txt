cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# Start a project.
project(exampleYarpTinyMath)

find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(exampleYarpTinyMath exampleYarpTinyMath.cpp)

target_link_libraries(exampleYarpTinyMath YARP::YARP_sig
                                          YARP::YARP_math
                                          YarpTinyMathLib)

include(GNUInstallDirs)

install(TARGETS exampleYarpTinyMath
        DESTINATION ${CMAKE_INSTALL_BINDIR})
