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

add_dependencies(CartesianControlClient COLOR_DEBUG)

target_link_libraries(CartesianControlClient YARP::YARP_OS
                                             YARP::YARP_dev)

yarp_install(TARGETS CartesianControlClient
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CartesianControlClient.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()