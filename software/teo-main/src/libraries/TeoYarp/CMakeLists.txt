# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

IF (ENABLE_TeoYarp)

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)
yarp_begin_plugin_library(TeoYarp)
add_subdirectory(ravepart)
add_subdirectory(ravepart2)
add_subdirectory(kdlcontroller)
yarp_end_plugin_library(TeoYarp)

install(TARGETS TeoYarp DESTINATION lib)

ENDIF (ENABLE_TeoYarp)

IF (ENABLE_testTeoYarp)
  yarp_add_plugin_library_executable(testTeoYarp TeoYarp)
  target_link_libraries(testTeoYarp ${YARP_LIBRARIES})
  install(TARGETS testTeoYarp DESTINATION bin)
ENDIF (ENABLE_testTeoYarp)

