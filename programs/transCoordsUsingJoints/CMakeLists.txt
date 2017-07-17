cmake_dependent_option(ENABLE_transCoordsUsingJoints "Enable/disable transCoordsUsingJoints program" ON
                       ENABLE_KdlVectorConverterLib OFF)

if(ENABLE_transCoordsUsingJoints)

# Find YARP.  Point the YARP_DIR environment variable at your build.
find_package(YARP REQUIRED)
find_package(orocos_kdl REQUIRED)

# Include any directories needed for YARP
include_directories(${YARP_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS})

# Set up our main executable.
add_executable(transCoordsUsingJoints main.cpp
                                      TransCoordsUsingJoints.cpp
                                      TransCoordsUsingJoints.hpp
                                      PremultPorts.cpp
                                      PremultPorts.hpp)

target_link_libraries(transCoordsUsingJoints YARP::YARP_OS
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             ${orocos_kdl_LIBRARIES}
                                             KdlVectorConverterLib)

install(TARGETS transCoordsUsingJoints
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
