# Create device interface target.
add_library(KinematicsDynamicsInterfaces INTERFACE)

# Configure usage requirements.
target_link_libraries(KinematicsDynamicsInterfaces INTERFACE YARP::YARP_os)

target_include_directories(KinematicsDynamicsInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Register interface headers.
set_property(TARGET KinematicsDynamicsInterfaces PROPERTY PUBLIC_HEADER ICartesianControl.h
                                                                        ICartesianSolver.h)

# Register export set.
install(TARGETS KinematicsDynamicsInterfaces
        EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS)

# Create alias target.
add_library(ROBOTICSLAB::KinematicsDynamicsInterfaces ALIAS KinematicsDynamicsInterfaces)

# Create devices.
add_subdirectory(AsibotSolver)
add_subdirectory(BasicCartesianControl)
add_subdirectory(CartesianControlClient)
add_subdirectory(CartesianControlServer)
add_subdirectory(CartesianControlServerROS2)
add_subdirectory(KdlSolver)
add_subdirectory(KdlTreeSolver)
