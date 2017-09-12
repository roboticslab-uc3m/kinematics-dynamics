option(ENABLE_haarDetectionController "Enable/disable haarDetectionController program" ON)

if(ENABLE_haarDetectionController)

# Find YARP.
find_package(YARP REQUIRED)

# Find ROBOTICSLAB_YARP_DEVICES.
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(haarDetectionController main.cpp
                                       HaarDetectionController.hpp
                                       HaarDetectionController.cpp
                                       GrabberResponder.hpp
                                       GrabberResponder.cpp)

add_dependencies(haarDetectionController COLOR_DEBUG)

target_link_libraries(haarDetectionController YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev)

install(TARGETS haarDetectionController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
