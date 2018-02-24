cmake_minimum_required(VERSION 2.8.0)

# Start a project.
project(exampleYarpTinyMath)

find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

# Set up our main executable.
add_executable(exampleYarpTinyMath exampleYarpTinyMath.cpp)

target_link_libraries(exampleYarpTinyMath YARP::YARP_OS
                                          YARP::YARP_init
                                          YARP::YARP_math
                                          YarpTinyMathLib)

include(GNUInstallDirs)

install(TARGETS exampleYarpTinyMath
        DESTINATION ${CMAKE_INSTALL_BINDIR})
