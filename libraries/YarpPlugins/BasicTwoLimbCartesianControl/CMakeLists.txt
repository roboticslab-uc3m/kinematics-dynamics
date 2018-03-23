# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(BasicTwoLimbCartesianControl
                    CATEGORY device
                    TYPE roboticslab::BasicTwoLimbCartesianControl
                    INCLUDE BasicTwoLimbCartesianControl.hpp
                    DEFAULT OFF
                    DEPENDS "ENABLE_TrajectoryLib;ENABLE_Gait")

if(NOT SKIP_BasicTwoLimbCartesianControl)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(BasicTwoLimbCartesianControl BasicTwoLimbCartesianControl.hpp
                                             DeviceDriverImpl.cpp
                                             ICartesianControlImpl.cpp
                                             RateThreadImpl.cpp)

add_dependencies(BasicTwoLimbCartesianControl COLOR_DEBUG)

target_link_libraries(BasicTwoLimbCartesianControl YARP::YARP_OS
                                                   YARP::YARP_dev
                                                   TrajectoryLib
                                                   Gait)

yarp_install(TARGETS BasicTwoLimbCartesianControl
             COMPONENT runtime
             LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES BasicTwoLimbCartesianControl.ini
             COMPONENT runtime
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
