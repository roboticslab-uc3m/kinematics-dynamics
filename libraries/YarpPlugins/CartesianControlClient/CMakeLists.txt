# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CartesianControlClient
                    CATEGORY device
                    TYPE roboticslab::CartesianControlClient
                    INCLUDE CartesianControlClient.hpp
                    DEFAULT ON)

if(NOT SKIP_CartesianControlClient)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(CartesianControlClient CartesianControlClient.hpp
                                       DeviceDriverImpl.cpp
                                       ICartesianControlImpl.cpp
                                       FkStreamResponder.cpp)

target_link_libraries(CartesianControlClient YARP::YARP_OS
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             KinematicsDynamicsInterfaces)

yarp_install(TARGETS CartesianControlClient
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CartesianControlClient.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
