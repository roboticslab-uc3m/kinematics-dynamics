# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

if(ENABLE_BasicTwoLimbCartesianControl)
    set(GAIT_PART_OF_PROJECT TRUE)
    add_subdirectory(gait)
endif()

add_subdirectory(KdlVectorConverterLib)
add_subdirectory(TrajectoryLib)
add_subdirectory(TeoYarp)
