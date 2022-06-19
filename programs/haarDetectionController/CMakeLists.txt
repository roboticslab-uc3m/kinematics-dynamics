if(NOT TARGET ROBOTICSLAB::YarpDevicesInterfaces AND (NOT DEFINED ENABLE_haarDetectionController OR ENABLE_haarDetectionController))
    message(WARNING "ROBOTICSLAB::YarpDevicesInterfaces target not found, disabling haarDetectionController program")
endif()

cmake_dependent_option(ENABLE_haarDetectionController "Enable/disable haarDetectionController program" ON
                       "TARGET ROBOTICSLAB::YarpDevicesInterfaces" OFF)

if(ENABLE_haarDetectionController)

    add_executable(haarDetectionController main.cpp
                                           HaarDetectionController.hpp
                                           HaarDetectionController.cpp
                                           GrabberResponder.hpp
                                           GrabberResponder.cpp
                                           LogComponent.hpp
                                           LogComponent.cpp)

    target_link_libraries(haarDetectionController YARP::YARP_os
                                                  YARP::YARP_init
                                                  YARP::YARP_dev
                                                  ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                  ROBOTICSLAB::YarpDevicesInterfaces)

    install(TARGETS haarDetectionController)

else()

    set(ENABLE_haarDetectionController OFF CACHE BOOL "Enable/disable haarDetectionController program" FORCE)

endif()
