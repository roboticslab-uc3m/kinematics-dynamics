option(ENABLE_streamingDeviceController "Enable/disable streamingDeviceController program" ON)

if(ENABLE_streamingDeviceController)

# Find YARP.
find_package(YARP REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(streamingDeviceController main.cpp
                                         StreamingDevice.hpp
                                         StreamingDevice.cpp
                                         SpnavSensorDevice.hpp
                                         SpnavSensorDevice.cpp
                                         LeapMotionSensorDevice.hpp
                                         LeapMotionSensorDevice.cpp
                                         WiimoteSensorDevice.hpp
                                         WiimoteSensorDevice.cpp
                                         StreamingDeviceController.hpp
                                         StreamingDeviceController.cpp)

add_dependencies(streamingDeviceController COLOR_DEBUG)

target_link_libraries(streamingDeviceController ${orocos_kdl_LIBRARIES}
                                                YARP::YARP_OS
                                                YARP::YARP_init
                                                YARP::YARP_dev
                                                YARP::YARP_sig)

install(TARGETS streamingDeviceController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
