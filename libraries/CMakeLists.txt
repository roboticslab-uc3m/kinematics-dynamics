# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

include(CMakeDependentOption)

# Try to find Gait library on system, don't fail otherwise.
find_package(ROBOTICSLAB_GAIT QUIET)

unset(HAVE_GAIT)

# Do we have Gait, either as separate installation or git submodule of current project?
if(ROBOTICSLAB_GAIT_FOUND OR EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/gait/CMakeLists.txt)
    set(HAVE_GAIT TRUE)
endif()

# Present GUI option to user if Gait was found.
cmake_dependent_option(ENABLE_Gait "Enable/disable Gait library" ON
                       HAVE_GAIT OFF)

# Gait enabled and not found on system, but available as submodule.
if(ENABLE_Gait AND NOT ROBOTICSLAB_GAIT_FOUND)
    message(STATUS "Using Gait library as part of current project")
    add_subdirectory(gait)
    set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS Gait)
endif()

add_subdirectory(KdlVectorConverterLib)
add_subdirectory(TrajectoryLib)
add_subdirectory(TeoYarp)
