# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT AMOR_API_FOUND AND (NOT DEFINED ENABLE_AmorCartesianControl OR ENABLE_AmorCartesianControl))
    message(WARNING "AMOR_API package not found, disabling AmorCartesianControl")
endif()

yarp_prepare_plugin(AmorCartesianControl
                    CATEGORY device
                    TYPE roboticslab::AmorCartesianControl
                    INCLUDE AmorCartesianControl.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_KinematicRepresentationLib;AMOR_API_FOUND"
                    EXTRA_CONFIG WRAPPER=CartesianControlServer)

if(NOT SKIP_AmorCartesianControl)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(AmorCartesianControl AmorCartesianControl.hpp
                                         AmorCartesianControl.cpp
                                         DeviceDriverImpl.cpp
                                         ICartesianControlImpl.cpp)

    target_link_libraries(AmorCartesianControl YARP::YARP_OS
                                               YARP::YARP_dev
                                               AMOR::amor_api
                                               ROBOTICSLAB::ColorDebug
                                               ROBOTICSLAB::KinematicRepresentationLib
                                               ROBOTICSLAB::KinematicsDynamicsInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS AmorCartesianControl
                     LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS AmorCartesianControl
                     LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES AmorCartesianControl.ini
                     DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

else()

    set(ENABLE_AmorCartesianControl OFF CACHE BOOL "Enable/disable AmorCartesianControl device" FORCE)

endif()
