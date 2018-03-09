option(ENABLE_haarDetectionController "Enable/disable haarDetectionController program" ON)

if(ENABLE_haarDetectionController)

# Find YARP.
find_package(YARP REQUIRED)

# Find ROBOTICSLAB_YARP_DEVICES.
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS})

# Set up our main executable.
add_executable(haarDetectionController main.cpp
                                       HaarDetectionController.hpp
                                       HaarDetectionController.cpp
                                       GrabberResponder.hpp
                                       GrabberResponder.cpp)

add_dependencies(haarDetectionController COLOR_DEBUG)

target_link_libraries(haarDetectionController YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              KinematicsDynamicsInterfaces
                                              ROBOTICSLAB::YarpDevicesInterfaces)

install(TARGETS haarDetectionController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
