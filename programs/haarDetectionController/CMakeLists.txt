option(ENABLE_haarDetectionController "Enable/disable haarDetectionController program" ON)

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
                                                  ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS haarDetectionController)

endif()
