if(NOT ROBOTICSLAB_YARP_DEVICES_FOUND AND (NOT DEFINED ENABLE_haarDetectionController OR ENABLE_haarDetectionController))
    message(WARNING "ROBOTICSLAB_YARP_DEVICES package not found, disabling haarDetectionController program")
endif()

cmake_dependent_option(ENABLE_haarDetectionController "Enable/disable haarDetectionController program" ON
                       ROBOTICSLAB_YARP_DEVICES_FOUND OFF)

if(ENABLE_haarDetectionController)

    add_executable(haarDetectionController main.cpp
                                           HaarDetectionController.hpp
                                           HaarDetectionController.cpp
                                           GrabberResponder.hpp
                                           GrabberResponder.cpp)

    target_link_libraries(haarDetectionController YARP::YARP_OS
                                                  YARP::YARP_init
                                                  YARP::YARP_dev
                                                  ROBOTICSLAB::ColorDebug
                                                  ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                  ROBOTICSLAB::YarpDevicesInterfaces)

    install(TARGETS haarDetectionController
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_haarDetectionController OFF CACHE BOOL "Enable/disable haarDetectionController program" FORCE)

endif()
