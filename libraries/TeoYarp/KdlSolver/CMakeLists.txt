# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_PLUGIN(KdlSolver CATEGORY device TYPE teo::KdlSolver INCLUDE KdlSolver.hpp)

IF (NOT SKIP_KdlSolver)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

YARP_ADD_PLUGIN(KdlSolver KdlSolver.hpp DeviceDriverImpl.cpp ICartesianSolverImpl.cpp)
TARGET_LINK_LIBRARIES(KdlSolver ${YARP_LIBRARIES} ${orocos_kdl_LIBRARIES})
install(TARGETS KdlSolver DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
# set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp KdlSolver ${orocos_kdl_LIBRARIES} CACHE INTERNAL "appended libraries")

#install(TARGETS KdlSolver DESTINATION lib)
  yarp_install(TARGETS KdlSolver
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES KdlSolver.ini
              COMPONENT runtime
              DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

ENDIF (NOT SKIP_KdlSolver)

