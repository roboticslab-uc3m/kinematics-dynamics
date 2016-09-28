# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Author: Juan G. Victores

YARP_PREPARE_DEVICE(FakeControlboardOR TYPE teo::FakeControlboardOR INCLUDE FakeControlboardOR.hpp WRAPPER controlboardwrapper2)

IF (NOT SKIP_FakeControlboardOR)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})  # kdl name depends on version

LINK_DIRECTORIES(${TEO_LINK_DIRS})

YARP_ADD_PLUGIN(FakeControlboardOR FakeControlboardOR.hpp DeviceDriverImpl.cpp IControlLimitsImpl.cpp IControlModeImpl.cpp IEncodersImpl.cpp IEncodersTimedImpl.cpp IPositionImpl.cpp IPosition2Impl.cpp IVelocityImpl.cpp IVelocity2Impl.cpp ITorqueImpl.cpp SharedArea.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(FakeControlboardOR ${YARP_LIBRARIES})

install(TARGETS FakeControlboardOR DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp FakeControlboardOR CACHE INTERNAL "appended libraries") 

ENDIF (NOT SKIP_FakeControlboardOR)

