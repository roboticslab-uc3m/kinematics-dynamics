# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(CartesianControlServer
                    CATEGORY device
                    TYPE roboticslab::CartesianControlServer
                    INCLUDE CartesianControlServer.hpp
                    WRAPPER CartesianControlServer
                    DEFAULT ON)

if(NOT SKIP_CartesianControlServer)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

yarp_add_plugin(CartesianControlServer CartesianControlServer.hpp
                                       CartesianControlServer.cpp
                                       DeviceDriverImpl.cpp)

target_link_libraries(CartesianControlServer ${YARP_LIBRARIES})

yarp_install(TARGETS CartesianControlServer
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES CartesianControlServer.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
