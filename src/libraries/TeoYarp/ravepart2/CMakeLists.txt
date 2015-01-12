# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores

YARP_PREPARE_DEVICE(ravepart2 TYPE teo::RavePart2 INCLUDE RavePart2.hpp WRAPPER controlboard)

IF (NOT SKIP_ravepart2)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})  # kdl name depends on version

LINK_DIRECTORIES(${TEO_LINK_DIRS})

YARP_ADD_PLUGIN(ravepart2 RavePart2.hpp IDeviceImpl.cpp IControlLimitsImpl.cpp IControlModeImpl.cpp IEncoderImpl.cpp IPosition2Impl.cpp IVelocity2Impl.cpp ITorqueImpl.cpp SharedArea.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(ravepart2 ${YARP_LIBRARIES})

install(TARGETS ravepart2 DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp ravepart2 CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_ravepart2)

