# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico

YARP_PREPARE_PLUGIN(FakeControlboardOR CATEGORY device TYPE teo::FakeControlboardOR INCLUDE FakeControlboardOR.hpp WRAPPER controlboardwrapper2)

IF (NOT SKIP_FakeControlboardOR)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenRAVE REQUIRED)
find_package(Boost COMPONENTS iostreams python thread system)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${OpenRAVE_INCLUDE_DIRS} ${TEO_INCLUDE_DIRS})  # kdl name depends on version
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

LINK_DIRECTORIES(${TEO_LINK_DIRS})

YARP_ADD_PLUGIN(FakeControlboardOR FakeControlboardOR.hpp DeviceDriverImpl.cpp IControlLimitsImpl.cpp IControlLimits2Impl.cpp IControlModeImpl.cpp IEncodersImpl.cpp IEncodersTimedImpl.cpp IPositionImpl.cpp IPosition2Impl.cpp IVelocityImpl.cpp IVelocity2Impl.cpp ITorqueImpl.cpp SharedArea.cpp RateThreadImpl.cpp)
set_target_properties(${KEYWORD} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXXFLAGS}")
set_target_properties(${KEYWORD} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
TARGET_LINK_LIBRARIES(FakeControlboardOR ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${YARP_LIBRARIES})

install(TARGETS FakeControlboardOR DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
# set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp FakeControlboardOR ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} CACHE INTERNAL "appended libraries") 

  #install(TARGETS FakeControlboardOR DESTINATION lib)
  yarp_install(TARGETS FakeControlboardOR
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES FakeControlboardOR.ini
              COMPONENT runtime
DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

ENDIF (NOT SKIP_FakeControlboardOR)

