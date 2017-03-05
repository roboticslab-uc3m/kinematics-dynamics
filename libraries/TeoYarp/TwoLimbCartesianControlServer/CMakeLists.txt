# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_PLUGIN(TwoLimbCartesianControlServer CATEGORY device TYPE teo::TwoLimbCartesianControlServer INCLUDE TwoLimbCartesianControlServer.hpp WRAPPER TwoLimbCartesianControlServer)

IF (NOT SKIP_TwoLimbCartesianControlServer)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})

YARP_ADD_PLUGIN(TwoLimbCartesianControlServer TwoLimbCartesianControlServer.hpp TwoLimbCartesianControlServer.cpp DeviceDriverImpl.cpp)
TARGET_LINK_LIBRARIES(TwoLimbCartesianControlServer ${YARP_LIBRARIES})
install(TARGETS TwoLimbCartesianControlServer DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
#set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp TwoLimbCartesianControlServer CACHE INTERNAL "appended libraries")

yarp_install(TARGETS TwoLimbCartesianControlServer
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES TwoLimbCartesianControlServer.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_TwoLimbCartesianControlServer)

