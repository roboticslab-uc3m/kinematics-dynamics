# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CartesianControlServer
                    CATEGORY device
                    TYPE roboticslab::CartesianControlServer
                    INCLUDE CartesianControlServer.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_KinematicRepresentationLib)

if(NOT SKIP_CartesianControlServer)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(CartesianControlServer CartesianControlServer.hpp
                                       DeviceDriverImpl.cpp
                                       PeriodicThreadImpl.cpp
                                       RpcResponder.cpp
                                       StreamResponder.cpp)

target_link_libraries(CartesianControlServer YARP::YARP_OS
                                             YARP::YARP_dev
                                             ROBOTICSLAB::ColorDebug
                                             KinematicRepresentationLib
                                             KinematicsDynamicsInterfaces)

yarp_install(TARGETS CartesianControlServer
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CartesianControlServer.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
