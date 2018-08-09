# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(AmorCartesianControl
                    CATEGORY device
                    TYPE roboticslab::AmorCartesianControl
                    INCLUDE AmorCartesianControl.hpp)

verbose_dependent_option(AmorCartesianControl DEPENDS_WARNING AMOR_API_FOUND
                                              DEPENDS_OTHER ENABLE_KinematicRepresentationLib)

if(ENABLE_AmorCartesianControl)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

    yarp_add_plugin(AmorCartesianControl AmorCartesianControl.hpp
                                         AmorCartesianControl.cpp
                                         DeviceDriverImpl.cpp
                                         ICartesianControlImpl.cpp)

    target_link_libraries(AmorCartesianControl YARP::YARP_OS
                                               YARP::YARP_dev
                                               AMOR::amor_api
                                               ROBOTICSLAB::ColorDebug
                                               KinematicRepresentationLib
                                               KinematicsDynamicsInterfaces)

    yarp_install(TARGETS AmorCartesianControl
                 COMPONENT runtime
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES AmorCartesianControl.ini
                 COMPONENT runtime
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
