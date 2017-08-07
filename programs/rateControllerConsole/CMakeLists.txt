option(ENABLE_rateControllerConsole "Enable/disable rateControllerConsole program" ON)

if(ENABLE_rateControllerConsole)

# Find YARP.
find_package(YARP REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(rateControllerConsole main.cpp
                                     RateControllerConsole.hpp
                                     RateControllerConsole.cpp)

add_dependencies(rateControllerConsole COLOR_DEBUG)

target_link_libraries(rateControllerConsole YARP::YARP_OS
                                            YARP::YARP_init
                                            YARP::YARP_dev)

install(TARGETS rateControllerConsole
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
