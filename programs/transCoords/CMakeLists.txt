find_package(orocos_kdl ${_target_kdl_version} QUIET)

if(ENABLE_transCoords AND NOT orocos_kdl_FOUND)
    message(WARNING "orocos_kdl package not found, disabling transCoords")
    set(ENABLE_transCoords OFF CACHE BOOL "Enable/disable transCoords program" FORCE)
    return()
endif()

cmake_dependent_option(ENABLE_transCoords "Enable/disable transCoords program" ON
                       "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib" OFF)

if(ENABLE_transCoords)

find_package(YARP REQUIRED)

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
