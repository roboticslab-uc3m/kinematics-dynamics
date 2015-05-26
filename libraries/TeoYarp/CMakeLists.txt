# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

IF (ENABLE_TeoYarp)

  option(ENABLE_launchTeoYarp "Choose if you want to compile launchTeoYarp" TRUE)

  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
  include(YarpPlugin)

  # Exporting dependencies for TEOConfig.cmake quite manually for now... 
  set(TEO_BODY_LIBRARIES ${TEO_LIBRARIES} TeoYarp CACHE INTERNAL "appended libraries") 

  yarp_begin_plugin_library(TeoYarp)
    add_subdirectory(RavePart)
    add_subdirectory(KdlController)
    add_subdirectory(KdlSolver)
  yarp_end_plugin_library(TeoYarp)

  install(TARGETS TeoYarp DESTINATION lib)

  IF (ENABLE_launchTeoYarp)

    yarp_add_plugin_library_executable(launchTeoYarp TeoYarp)
    install(TARGETS launchTeoYarp DESTINATION bin)

  ENDIF (ENABLE_launchTeoYarp)

ENDIF (ENABLE_TeoYarp)


