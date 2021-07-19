if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_keyboardController OR ENABLE_keyboardController))
    message(WARNING "orocos_kdl package not found, disabling keyboardController")
endif()

cmake_dependent_option(ENABLE_keyboardController "Enable/disable keyboardController program" ON
                       "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib;orocos_kdl_FOUND" OFF)

if(ENABLE_keyboardController)

    add_executable(keyboardController main.cpp
                                      KeyboardController.hpp
                                      KeyboardController.cpp
                                      LinearTrajectoryThread.hpp
                                      LinearTrajectoryThread.cpp
                                      LogComponent.hpp
                                      LogComponent.cpp)

    target_link_libraries(keyboardController ${orocos_kdl_LIBRARIES}
                                             YARP::YARP_os
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             ROBOTICSLAB::KdlVectorConverterLib
                                             ROBOTICSLAB::KinematicRepresentationLib
                                             ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(keyboardController PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS keyboardController
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
