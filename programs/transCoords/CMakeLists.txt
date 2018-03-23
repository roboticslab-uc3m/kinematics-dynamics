find_package(orocos_kdl ${_target_kdl_version} QUIET)

if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_transCoords OR ENABLE_transCoords))
    message(WARNING "orocos_kdl package not found, disabling transCoords")
endif()

cmake_dependent_option(ENABLE_transCoords "Enable/disable transCoords program" ON
                       "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib;orocos_kdl_FOUND" OFF)

if(ENABLE_transCoords)

    # Include any directories needed for this target.
    include_directories(${YARP_INCLUDE_DIRS}
                        ${orocos_kdl_INCLUDE_DIRS})

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
                                      KinematicRepresentationLib
                                      KinematicsDynamicsInterfaces)

    install(TARGETS transCoords
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_transCoords OFF CACHE BOOL "Enable/disable transCoords program" FORCE)

endif()
