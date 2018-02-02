# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(AmorCartesianControl
                    CATEGORY device
                    TYPE roboticslab::AmorCartesianControl
                    INCLUDE AmorCartesianControl.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_KinematicRepresentationLib)

find_package(AMOR_API QUIET)

if(NOT SKIP_AmorCartesianControl)

if(NOT AMOR_API_FOUND)
    message(WARNING "AMOR_API package not found, disabling AmorCartesianControl")
    set(ENABLE_AmorCartesianControl OFF CACHE BOOL "Enable/disable AmorCartesianControl device" FORCE)
    return()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(AmorCartesianControl AmorCartesianControl.hpp
                                     AmorCartesianControl.cpp
                                     DeviceDriverImpl.cpp
                                     ICartesianControlImpl.cpp)

add_dependencies(AmorCartesianControl COLOR_DEBUG)

target_link_libraries(AmorCartesianControl YARP::YARP_OS
                                           YARP::YARP_dev
                                           AMOR::amor_api
                                           KinematicRepresentationLib)

yarp_install(TARGETS AmorCartesianControl
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES AmorCartesianControl.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
