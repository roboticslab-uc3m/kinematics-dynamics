# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(BasicCartesianControl
                    CATEGORY device
                    TYPE roboticslab::BasicCartesianControl
                    INCLUDE BasicCartesianControl.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_TrajectoryLib
                    EXTRA_CONFIG WRAPPER=CartesianControlServer)

if(NOT SKIP_BasicCartesianControl)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(BasicCartesianControl BasicCartesianControl.hpp
                                          BasicCartesianControl.cpp
                                          DeviceDriverImpl.cpp
                                          ICartesianControlImpl.cpp
                                          PeriodicThreadImpl.cpp)

    target_link_libraries(BasicCartesianControl YARP::YARP_OS
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug
                                                TrajectoryLib
                                                KinematicsDynamicsInterfaces)

    yarp_install(TARGETS BasicCartesianControl
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
