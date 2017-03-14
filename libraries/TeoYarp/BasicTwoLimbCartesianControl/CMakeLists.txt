# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_PLUGIN(BasicTwoLimbCartesianControl CATEGORY device TYPE teo::BasicTwoLimbCartesianControl INCLUDE BasicTwoLimbCartesianControl.hpp WRAPPER TwoLimbCartesianControlServer)

IF (NOT SKIP_BasicTwoLimbCartesianControl)

SET(CMAKE_MODULE_PATH ${ROBOTICSLAB_KINEMATICS_DYNAMICS_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${GAIT_INCLUDE_DIRS})

YARP_ADD_PLUGIN(BasicTwoLimbCartesianControl BasicTwoLimbCartesianControl.hpp DeviceDriverImpl.cpp ICartesianControlImpl.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(BasicTwoLimbCartesianControl ${YARP_LIBRARIES} ${KINEMATICS_DYNAMICS_LIBRARIES} ${GAIT_LIBRARIES})
install(TARGETS BasicTwoLimbCartesianControl DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(KINEMATICS_DYNAMICS_INCLUDE_DIRS ${KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
#set(KINEMATICS_DYNAMICS_LIBRARIES ${KINEMATICS_DYNAMICS_LIBRARIES} TeoYarp BasicTwoLimbCartesianControl CACHE INTERNAL "appended libraries")

yarp_install(TARGETS BasicTwoLimbCartesianControl
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES BasicTwoLimbCartesianControl.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_BasicTwoLimbCartesianControl)

