cmake_dependent_option(ENABLE_transCoords "Enable/disable transCoords program" ON
                       "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib;orocos_kdl_FOUND" OFF)

unset(ENABLE_transCoords)

if(NOT DEFINED ENABLE_transCoords)
    set(ENABLE_transCoords ON CACHE BOOL "")
endif()

if(NOT orocos_kdl_FOUND)
    if(ENABLE_transCoords)
        message(WARNING "orocos_kdl package not found, disabling transCoords")
    endif()
    if(NOT DEFINED ENABLE_transCoords-STORED)
        set(ENABLE_transCoords-STORED "${ENABLE_transCoords}" CACHE INTERNAL "Stored value of ENABLE_transCoords prior to forcing OFF")
    endif()
    set(ENABLE_transCoords OFF CACHE BOOL "Enable/disable transCoords" FORCE)
elseif(DEFINED ENABLE_transCoords-STORED AND NOT ENABLE_transCoords)
    set(ENABLE_transCoords "${ENABLE_transCoords-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_transCoords-STORED CACHE)
endif()

if(ENABLE_transCoords)

    # Include any directories needed for this target.
    include_directories(${orocos_kdl_INCLUDE_DIRS})

    # Set up our main executable.
    add_executable(transCoords main.cpp
                               TransCoords.cpp
                               TransCoords.hpp)

    target_link_libraries(transCoords YARP::YARP_OS
                                      YARP::YARP_init
                                      YARP::YARP_dev
                                      ${orocos_kdl_LIBRARIES}
                                      ROBOTICSLAB::ColorDebug
                                      KdlVectorConverterLib
                                      KinematicRepresentationLib
                                      KinematicsDynamicsInterfaces)

    install(TARGETS transCoords
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
