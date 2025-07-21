if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_streamingDeviceController OR ENABLE_streamingDeviceController))
    message(WARNING "orocos_kdl package not found, disabling streamingDeviceController")
endif()

cmake_dependent_option(ENABLE_streamingDeviceController "Enable/disable streamingDeviceController program" ON
                       "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND" OFF)

if(ENABLE_streamingDeviceController)

    # Include any directories needed for this target.
    include_directories(${orocos_kdl_INCLUDE_DIRS})

    # Set up our main executable.
    add_executable(streamingDeviceController main.cpp
                                             StreamingDevice.hpp
                                             StreamingDevice.cpp
                                             SpnavSensorDevice.hpp
                                             SpnavSensorDevice.cpp
                                            #  LeapMotionSensorDevice.hpp
                                            #  LeapMotionSensorDevice.cpp
                                             WiimoteDevice.hpp
                                             WiimoteDevice.cpp
                                             StreamingDeviceController.hpp
                                             StreamingDeviceController.cpp
                                             CentroidTransform.hpp
                                             CentroidTransform.cpp
                                             LogComponent.hpp
                                             LogComponent.cpp)

    target_link_libraries(streamingDeviceController ${orocos_kdl_LIBRARIES}
                                                    YARP::YARP_os
                                                    YARP::YARP_init
                                                    YARP::YARP_dev
                                                    YARP::YARP_sig
                                                    ROBOTICSLAB::KdlVectorConverterLib
                                                    ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS streamingDeviceController)

else()

    set(ENABLE_streamingDeviceController OFF CACHE BOOL "Enable/disable streamingDeviceController program" FORCE)

endif()
