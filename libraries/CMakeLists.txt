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
    set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} CACHE INTERNAL "appended header dirs")
ENDFOREACH()

IF (ENABLE_BasicTwoLimbCartesianControl)
    set(GAIT_PART_OF_PROJECT TRUE)
    add_subdirectory(gait)
ENDIF (ENABLE_BasicTwoLimbCartesianControl)

add_subdirectory(KdlVectorConverterLib)
add_subdirectory(TrajectoryLib)

add_subdirectory(TeoYarp)

