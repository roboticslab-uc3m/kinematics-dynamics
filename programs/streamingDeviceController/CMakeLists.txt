cmake_dependent_option(ENABLE_streamingDeviceController "Enable/disable streamingDeviceController program" ON
                       ENABLE_KdlVectorConverterLib OFF)

find_package(orocos_kdl ${_target_kdl_version} QUIET)

if(ENABLE_streamingDeviceController)

find_package(YARP REQUIRED)

if(NOT orocos_kdl_FOUND)
    message(WARNING "orocos_kdl package not found, disabling streamingDeviceController")
    set(ENABLE_streamingDeviceController OFF CACHE BOOL "Enable/disable streamingDeviceController program" FORCE)
    return()
endif()

find_package(ROBOTICSLAB_YARP_DEVICES QUIET)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
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

if(ROBOTICSLAB_YARP_DEVICES_FOUND)
    message(STATUS "ROBOTICSLAB_YARP_DEVICES found, building streamingDeviceController with sensor support")
    target_include_directories(streamingDeviceController PRIVATE ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS})
    target_compile_definitions(streamingDeviceController PRIVATE SDC_WITH_SENSORS)
else()
    message(STATUS "ROBOTICSLAB_YARP_DEVICES not found, building streamingDeviceController without sensor support")
endif()

target_link_libraries(streamingDeviceController ${orocos_kdl_LIBRARIES}
                                                YARP::YARP_OS
                                                YARP::YARP_init
                                                YARP::YARP_dev
                                                YARP::YARP_sig
                                                KdlVectorConverterLib)

install(TARGETS streamingDeviceController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
