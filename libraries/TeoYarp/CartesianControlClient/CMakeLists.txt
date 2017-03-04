# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_PLUGIN(CartesianControlClient CATEGORY device TYPE teo::CartesianControlClient INCLUDE CartesianControlClient.hpp)

IF (NOT SKIP_CartesianControlClient)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})

YARP_ADD_PLUGIN(CartesianControlClient CartesianControlClient.hpp DeviceDriverImpl.cpp ICartesianControlImpl.cpp)
TARGET_LINK_LIBRARIES(CartesianControlClient ${YARP_LIBRARIES})
install(TARGETS CartesianControlClient DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
# set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp CartesianControlClient CACHE INTERNAL "appended libraries")

#install(TARGETS CartesianControlClient DESTINATION lib)
  yarp_install(TARGETS CartesianControlClient
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES CartesianControlClient.ini
              COMPONENT runtime
DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_CartesianControlClient)

