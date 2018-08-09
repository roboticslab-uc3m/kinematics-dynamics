verbose_dependent_option(transCoords DEPENDS_WARNING orocos_kdl_FOUND
                                     DEPENDS_OTHER ENABLE_KdlVectorConverterLib
                                                   ENABLE_KinematicRepresentationLib)

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
