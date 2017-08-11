cmake_dependent_option(ENABLE_transCoordsUsingJoints "Enable/disable transCoordsUsingJoints program" ON
                       ENABLE_KdlVectorConverterLib OFF)

if(ENABLE_transCoordsUsingJoints)

# Find YARP and Orocos KDL.
find_package(YARP REQUIRED)
find_package(orocos_kdl REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(transCoordsUsingJoints main.cpp
                                      TransCoordsUsingJoints.cpp
                                      TransCoordsUsingJoints.hpp
                                      PremultPorts.cpp
                                      PremultPorts.hpp)

add_dependencies(transCoordsUsingJoints ROBOTICSLAB::COLOR_DEBUG)

target_link_libraries(transCoordsUsingJoints YARP::YARP_OS
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             ${orocos_kdl_LIBRARIES}
                                             KdlVectorConverterLib)

install(TARGETS transCoordsUsingJoints
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
