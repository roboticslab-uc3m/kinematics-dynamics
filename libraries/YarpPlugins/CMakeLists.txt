# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

if(YARP_VERSION_SHORT VERSION_LESS 2.3.70)
    include_directories(${YARP_INCLUDE_DIRS})
endif()

# Create device interface target.
add_library(KinematicsDynamicsInterfaces INTERFACE)

# Configure usage requirements.
target_include_directories(KinematicsDynamicsInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install interface headers.
install(FILES ICartesianControl.h
              ICartesianSolver.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Register export set.
install(TARGETS KinematicsDynamicsInterfaces
        EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS)

# Store target in the global scope.
set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS KinematicsDynamicsInterfaces)

# Create devices.
add_subdirectory(AmorCartesianControl)
add_subdirectory(AsibotSolver)
add_subdirectory(BasicCartesianControl)
#add_subdirectory(BasicTwoLimbCartesianControl)
add_subdirectory(CartesianControlClient)
add_subdirectory(CartesianControlServer)
add_subdirectory(KdlSolver)
#add_subdirectory(TwoLimbCartesianControlServer)
