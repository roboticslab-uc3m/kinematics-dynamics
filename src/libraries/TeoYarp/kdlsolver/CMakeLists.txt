# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_DEVICE(kdlcontroller TYPE teo::KdlController INCLUDE KdlController.hpp)

IF (NOT SKIP_kdlcontroller)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

YARP_ADD_PLUGIN(kdlcontroller KdlController.hpp IDeviceImpl.cpp ICartesianImpl.cpp RateThreadImpl.cpp HelperFuncs.cpp)
TARGET_LINK_LIBRARIES(kdlcontroller ${YARP_LIBRARIES} ${orocos_kdl_LIBRARIES})
install(TARGETS kdlcontroller DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp kdlcontroller ${orocos_kdl_LIBRARIES} CACHE INTERNAL "appended libraries")

ENDIF (NOT SKIP_kdlcontroller)

