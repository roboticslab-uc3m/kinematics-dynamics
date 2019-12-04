cmake_dependent_option(ENABLE_keyboardController "Enable/disable keyboardController program" ON
                       ENABLE_KinematicRepresentationLib OFF)

if(ENABLE_keyboardController)

# Set up our main executable.
add_executable(keyboardController main.cpp
                                  KeyboardController.hpp
                                  KeyboardController.cpp
                                  LinearTrajectoryThread.hpp
                                  LinearTrajectoryThread.cpp)

target_link_libraries(keyboardController YARP::YARP_OS
                                         YARP::YARP_init
                                         YARP::YARP_dev
                                         ROBOTICSLAB::ColorDebug
                                         KinematicRepresentationLib
                                         KinematicsDynamicsInterfaces
                                         TrajectoryLib)

install(TARGETS keyboardController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
