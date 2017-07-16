option(ENABLE_transCoordsUsingJoints "Enable/disable compilation of transCoordsUsingJoints" ON)

if(ENABLE_transCoordsUsingJoints)

# Find YARP.  Point the YARP_DIR environment variable at your build.
find_package(YARP REQUIRED)
find_package(orocos_kdl REQUIRED)

# Inclue any directories needed for YARP
include_directories(${YARP_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

# Set up our main executable.
add_executable(transCoordsUsingJoints main.cpp
                                      TransCoordsUsingJoints.cpp
                                      TransCoordsUsingJoints.hpp
                                      PremultPorts.cpp
                                      PremultPorts.hpp)

target_link_libraries(transCoordsUsingJoints ${YARP_LIBRARIES}
                                             ${orocos_kdl_LIBRARIES}
                                             KdlVectorConverterLib)

install(TARGETS transCoordsUsingJoints
        DESTINATION bin)

endif()
