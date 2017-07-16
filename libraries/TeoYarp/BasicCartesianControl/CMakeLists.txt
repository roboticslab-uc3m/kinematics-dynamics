# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(BasicCartesianControl
                    CATEGORY device
                    TYPE roboticslab::BasicCartesianControl
                    INCLUDE BasicCartesianControl.hpp
                    WRAPPER CartesianControlServer
                    DEFAULT ON)

if(NOT SKIP_BasicCartesianControl)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

yarp_add_plugin(BasicCartesianControl BasicCartesianControl.hpp
                                      DeviceDriverImpl.cpp
                                      ICartesianControlImpl.cpp
                                      RateThreadImpl.cpp)

target_link_libraries(BasicCartesianControl ${YARP_LIBRARIES}
                                            ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES})

yarp_install(TARGETS BasicCartesianControl
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES BasicCartesianControl.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
