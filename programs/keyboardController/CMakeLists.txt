verbose_dependent_option(keyboardController DEPENDS_OTHER ENABLE_KinematicRepresentationLib)

if(ENABLE_keyboardController)

    add_executable(keyboardController main.cpp
                                      KeyboardController.hpp
                                      KeyboardController.cpp)

    target_link_libraries(keyboardController YARP::YARP_OS
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             KinematicRepresentationLib
                                             KinematicsDynamicsInterfaces)

    install(TARGETS keyboardController
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
