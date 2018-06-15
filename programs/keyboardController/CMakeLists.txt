cmake_dependent_option(ENABLE_keyboardController "Enable/disable keyboardController program" ON
                       ENABLE_KinematicRepresentationLib OFF)

if(ENABLE_keyboardController)

# Include any directories needed for this target.
if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    include_directories(${YARP_INCLUDE_DIRS})
endif()

# Set up our main executable.
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
