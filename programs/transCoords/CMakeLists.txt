cmake_dependent_option(ENABLE_transCoords "Enable/disable transCoords program" ON
                       "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib" OFF)

if(ENABLE_transCoords)

# Find YARP and Orocos KDL.
find_package(YARP REQUIRED)
find_package(orocos_kdl REQUIRED)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

# Include any directories needed for this target.
include_directories(${YARP_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${_common_includes})

# Set up our main executable.
add_executable(transCoords main.cpp
                           TransCoords.cpp
                           TransCoords.hpp)

add_dependencies(transCoords COLOR_DEBUG)

target_link_libraries(transCoords YARP::YARP_OS
                                  YARP::YARP_init
                                  YARP::YARP_dev
                                  ${orocos_kdl_LIBRARIES}
                                  KdlVectorConverterLib
                                  KinematicRepresentationLib)

install(TARGETS transCoords
        DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
