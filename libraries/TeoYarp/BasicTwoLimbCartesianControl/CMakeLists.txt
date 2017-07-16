# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(BasicTwoLimbCartesianControl
                    CATEGORY device
                    TYPE roboticslab::BasicTwoLimbCartesianControl
                    INCLUDE BasicTwoLimbCartesianControl.hpp
                    WRAPPER TwoLimbCartesianControlServer
                    DEFAULT ON)

if(NOT SKIP_BasicTwoLimbCartesianControl)

find_package(YARP REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS}
                    ${GAIT_INCLUDE_DIRS})

yarp_add_plugin(BasicTwoLimbCartesianControl BasicTwoLimbCartesianControl.hpp
                                             DeviceDriverImpl.cpp
                                             ICartesianControlImpl.cpp
                                             RateThreadImpl.cpp)

target_link_libraries(BasicTwoLimbCartesianControl ${YARP_LIBRARIES}
                                                   ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES}
                                                   ${GAIT_LIBRARIES})

# Exporting dependencies for TEOConfig.cmake quite manually for now...
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")

yarp_install(TARGETS BasicTwoLimbCartesianControl
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES BasicTwoLimbCartesianControl.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
