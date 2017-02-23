# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_PLUGIN(CartesianControlServer CATEGORY device TYPE teo::CartesianControlServer INCLUDE CartesianControlServer.hpp WRAPPER CartesianControlServer)

IF (NOT SKIP_CartesianControlServer)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})

YARP_ADD_PLUGIN(CartesianControlServer CartesianControlServer.hpp CartesianControlServer.cpp DeviceDriverImpl.cpp)
TARGET_LINK_LIBRARIES(CartesianControlServer ${YARP_LIBRARIES})
install(TARGETS CartesianControlServer DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
# set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp CartesianControlServer CACHE INTERNAL "appended libraries")

  #install(TARGETS CartesianControlServer DESTINATION lib)
  yarp_install(TARGETS CartesianControlServer
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES CartesianControlServer.ini
              COMPONENT runtime
DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_CartesianControlServer)

