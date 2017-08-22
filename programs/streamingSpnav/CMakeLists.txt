option(ENABLE_streamingSpnav "Enable/disable streamingSpnav program" ON)

if(ENABLE_streamingSpnav)

# Find YARP.
find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(streamingSpnav main.cpp
                              StreamingSpnav.hpp
                              StreamingSpnav.cpp)

add_dependencies(streamingSpnav COLOR_DEBUG)

target_link_libraries(streamingSpnav YARP::YARP_OS
                                     YARP::YARP_init
                                     YARP::YARP_dev
                                     YARP::YARP_sig
                                     YARP::YARP_math)

install(TARGETS streamingSpnav
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
