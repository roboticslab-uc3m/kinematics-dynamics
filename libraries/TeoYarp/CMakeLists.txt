# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

# Find YARP (common dependency for all devices).
find_package(YARP REQUIRED)

# Include YARP CMake modules (needless since YARP 2.3.70).
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# Utilities for creating YARP devices.
include(YarpPlugin)

# Common YARP includes for subdirectories (needless since YARP 2.3.70).
include_directories(${YARP_INCLUDE_DIRS})

# Include export headers (ATM only deprecation notices).
include_directories(${CMAKE_BINARY_DIR}/generated_include)

# Make interface headers discoverable by subdirectories.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(AmorCartesianControl)
add_subdirectory(AsibotSolver)
add_subdirectory(BasicCartesianControl)
add_subdirectory(BasicTwoLimbCartesianControl)
add_subdirectory(CartesianControlClient)
add_subdirectory(CartesianControlServer)
add_subdirectory(KdlSolver)
add_subdirectory(TwoLimbCartesianControlServer)

# Install interface headers.
install(FILES ICartesianControl.h
              ICartesianSolver.h
              ITwoLimbCartesianControl.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export variables to the global scope.
set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
