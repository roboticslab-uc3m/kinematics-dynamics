option(ENABLE_keyboardController "Enable/disable keyboardController program" ON)

if(ENABLE_keyboardController)

# Find YARP.
find_package(YARP REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(keyboardController main.cpp
                                  KeyboardController.hpp
                                  KeyboardController.cpp)

add_dependencies(keyboardController COLOR_DEBUG)

target_link_libraries(keyboardController YARP::YARP_OS
                                         YARP::YARP_init
                                         YARP::YARP_dev)

install(TARGETS keyboardController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
