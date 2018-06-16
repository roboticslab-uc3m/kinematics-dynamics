option(ENABLE_haarDetectionController "Enable/disable haarDetectionController program" ON)

if(ENABLE_haarDetectionController)

# Find ROBOTICSLAB_YARP_DEVICES.
find_package(ROBOTICSLAB_YARP_DEVICES REQUIRED)

# Include any directories needed for this target.
if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    include_directories(${YARP_INCLUDE_DIRS})
endif()

# Set up our main executable.
add_executable(haarDetectionController main.cpp
                                       HaarDetectionController.hpp
                                       HaarDetectionController.cpp
                                       GrabberResponder.hpp
                                       GrabberResponder.cpp)

target_link_libraries(haarDetectionController YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              ROBOTICSLAB::ColorDebug
                                              KinematicsDynamicsInterfaces
                                              ROBOTICSLAB::YarpDevicesInterfaces)

install(TARGETS haarDetectionController
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
