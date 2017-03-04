# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico

YARP_PREPARE_PLUGIN(FakeControlboard CATEGORY device TYPE teo::FakeControlboard INCLUDE FakeControlboard.hpp WRAPPER controlboardwrapper2)

IF (NOT SKIP_FakeControlboard)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})  # kdl name depends on version

LINK_DIRECTORIES(${TEO_LINK_DIRS})

YARP_ADD_PLUGIN(FakeControlboard FakeControlboard.hpp DeviceDriverImpl.cpp IControlLimitsImpl.cpp IControlModeImpl.cpp IEncodersImpl.cpp IEncodersTimedImpl.cpp IPositionImpl.cpp IPosition2Impl.cpp IVelocityImpl.cpp IVelocity2Impl.cpp ITorqueImpl.cpp SharedArea.cpp RateThreadImpl.cpp)
TARGET_LINK_LIBRARIES(FakeControlboard ${YARP_LIBRARIES})

install(TARGETS FakeControlboard DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
# set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp FakeControlboard CACHE INTERNAL "appended libraries") 

  #install(TARGETS FakeControlboard DESTINATION lib)
  yarp_install(TARGETS FakeControlboard
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES FakeControlboard.ini
              COMPONENT runtime
DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_FakeControlboard)

