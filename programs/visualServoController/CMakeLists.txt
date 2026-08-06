option(ENABLE_visualServoController "Enable/disable visualServoController program" ON)

if(ENABLE_visualServoController)

    add_executable(visualServoController main.cpp
                                         VisualServoController.hpp
                                         VisualServoController.cpp
                                         GrabberResponder.hpp
                                         GrabberResponder.cpp
                                         LogComponent.hpp
                                         LogComponent.cpp)

    target_link_libraries(visualServoController YARP::YARP_os
                                                YARP::YARP_init
                                                YARP::YARP_dev
                                                ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS visualServoController)

endif()
