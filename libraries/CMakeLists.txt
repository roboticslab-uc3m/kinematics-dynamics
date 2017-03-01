# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        SET(dirlist ${dirlist} ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
FOREACH(subdir ${SUBDIRS})
    set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} CACHE INTERNAL "appended header dirs")
ENDFOREACH()

include_directories("${CMAKE_BINARY_DIR}/install/include")

set(GAIT_PART_OF_PROJECT TRUE)
add_subdirectory(gait)

add_subdirectory(KdlVectorConverterLib)
add_subdirectory(TrajectoryLib)

add_subdirectory(TeoYarp)

