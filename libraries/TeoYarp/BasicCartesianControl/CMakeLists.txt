# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_PLUGIN(BasicCartesianControl CATEGORY device TYPE teo::BasicCartesianControl INCLUDE BasicCartesianControl.hpp WRAPPER CartesianControlServer)

IF (NOT SKIP_BasicCartesianControl)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})

YARP_ADD_PLUGIN(BasicCartesianControl BasicCartesianControl.hpp DeviceDriverImpl.cpp ICartesianControlImpl.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(BasicCartesianControl ${YARP_LIBRARIES} ${TEO_LIBRARIES})
install(TARGETS BasicCartesianControl DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
# set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp BasicCartesianControl CACHE INTERNAL "appended libraries")

  #install(TARGETS BasicCartesianControl DESTINATION lib)
  yarp_install(TARGETS BasicCartesianControl
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES BasicCartesianControl.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_BasicCartesianControl)

