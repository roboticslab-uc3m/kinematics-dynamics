verbose_dependent_option(streamingDeviceController DEPENDS_WARNING orocos_kdl_FOUND
                                                   DEPENDS_OTHER ENABLE_KdlVectorConverterLib)

if(ENABLE_streamingDeviceController)

    # Include any directories needed for this target.
    include_directories(${orocos_kdl_INCLUDE_DIRS})

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

    target_link_libraries(streamingDeviceController ${orocos_kdl_LIBRARIES}
                                                    YARP::YARP_OS
                                                    YARP::YARP_init
                                                    YARP::YARP_dev
                                                    YARP::YARP_sig
                                                    ROBOTICSLAB::ColorDebug
                                                    KdlVectorConverterLib
                                                    KinematicsDynamicsInterfaces)

    if(ROBOTICSLAB_YARP_DEVICES_FOUND)
        message(STATUS "ROBOTICSLAB_YARP_DEVICES found, building streamingDeviceController with sensor support")
        target_link_libraries(streamingDeviceController ROBOTICSLAB::YarpDevicesInterfaces)
        target_compile_definitions(streamingDeviceController PRIVATE SDC_WITH_SENSORS)
    else()
        message(STATUS "ROBOTICSLAB_YARP_DEVICES not found, building streamingDeviceController without sensor support")
    endif()

    install(TARGETS streamingDeviceController
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
