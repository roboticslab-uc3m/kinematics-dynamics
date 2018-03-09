cmake_dependent_option(ENABLE_keyboardController "Enable/disable keyboardController program" ON
                       ENABLE_KinematicRepresentationLib OFF)

if(ENABLE_keyboardController)

# Find YARP.
find_package(YARP REQUIRED)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS})

# Set up our main executable.
add_executable(keyboardController main.cpp
                                  KeyboardController.hpp
                                  KeyboardController.cpp)

add_dependencies(keyboardController COLOR_DEBUG)

target_link_libraries(keyboardController YARP::YARP_OS
                                         YARP::YARP_init
                                         YARP::YARP_dev
                                         KinematicRepresentationLib
                                         KinematicsDynamicsInterfaces)

install(TARGETS keyboardController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
