# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

YARP_PREPARE_DEVICE(BasicTwoLimbCartesianControl TYPE teo::BasicTwoLimbCartesianControl INCLUDE BasicTwoLimbCartesianControl.hpp WRAPPER CartesianControlServer)

IF (NOT SKIP_BasicTwoLimbCartesianControl)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})

YARP_ADD_PLUGIN(BasicTwoLimbCartesianControl BasicTwoLimbCartesianControl.hpp DeviceDriverImpl.cpp ICartesianControlImpl.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(BasicTwoLimbCartesianControl ${YARP_LIBRARIES} ${TEO_LIBRARIES})
install(TARGETS BasicTwoLimbCartesianControl DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp BasicTwoLimbCartesianControl CACHE INTERNAL "appended libraries")

ENDIF (NOT SKIP_BasicTwoLimbCartesianControl)

