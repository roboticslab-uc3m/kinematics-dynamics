cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Start a project.
project(exampleYarpTinyMath LANGUAGES CXX)

find_package(YARP 3.0 REQUIRED COMPONENTS sig math)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(exampleYarpTinyMath exampleYarpTinyMath.cpp)

target_link_libraries(exampleYarpTinyMath YARP::YARP_sig
                                          YARP::YARP_math
                                          ROBOTICSLAB::YarpTinyMathLib)

include(GNUInstallDirs)

install(TARGETS exampleYarpTinyMath
        DESTINATION ${CMAKE_INSTALL_BINDIR})
