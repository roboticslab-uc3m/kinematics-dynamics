# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores

YARP_PREPARE_DEVICE(ravepart TYPE teo::RavePart INCLUDE RavePart.hpp WRAPPER controlboard)

IF (NOT SKIP_ravepart)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})  # kdl name depends on version

LINK_DIRECTORIES(${TEO_LINK_DIRS})

YARP_ADD_PLUGIN(ravepart RavePart.hpp IDeviceImpl.cpp IControlLimitsImpl.cpp IControlModeImpl.cpp IEncoderImpl.cpp IPositionImpl.cpp IVelocityImpl.cpp ITorqueImpl.cpp SharedArea.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(ravepart ${YARP_LIBRARIES})

install(TARGETS ravepart DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp ravepart CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_ravepart)

