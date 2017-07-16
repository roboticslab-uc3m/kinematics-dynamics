# Copyright: Universidad Carlos III de Madrid (C) 2017;
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)

project(ROBOTICSLAB_KINEMATICS_DYNAMICS)

set(BUILD_SHARED_LIBS TRUE CACHE INTERNAL "hide this!")

##### test and coverage options
option(ENABLE_tests "Choose if you want to compile tests" TRUE)
option(ENABLE_coverage "Choose if you want to enable coverage collection" FALSE)

# ColorDebug options
option(COLOR_DEBUG_FULL_FILE "Choose if you want to compile with CD_FULL_FILE" FALSE)
if(COLOR_DEBUG_FULL_FILE)
    add_definitions(-DCD_FULL_FILE)
endif()

option(COLOR_DEBUG_HIDE_ERROR "Choose if you want to compile with CD_HIDE_ERROR" FALSE)
if(COLOR_DEBUG_HIDE_ERROR)
    add_definitions(-DCD_HIDE_ERROR)
endif()

option(COLOR_DEBUG_HIDE_WARNING "Choose if you want to compile with CD_HIDE_WARNING" FALSE)
if(COLOR_DEBUG_HIDE_WARNING)
    add_definitions(-DCD_HIDE_WARNING)
endif()

option(COLOR_DEBUG_HIDE_SUCCESS "Choose if you want to compile with CD_HIDE_SUCCESS" FALSE)
if(COLOR_DEBUG_HIDE_SUCCESS)
    add_definitions(-DCD_HIDE_SUCCESS)
endif()

option(COLOR_DEBUG_HIDE_INFO "Choose if you want to compile with CD_HIDE_INFO" FALSE)
if(COLOR_DEBUG_HIDE_INFO)
    add_definitions(-DCD_HIDE_INFO)
endif()

option(COLOR_DEBUG_HIDE_DEBUG "Choose if you want to compile with CD_HIDE_DEBUG" FALSE)
if(COLOR_DEBUG_HIDE_DEBUG)
    add_definitions(-DCD_HIDE_DEBUG)
endif()

if(MSVC)
  message(STATUS "Running on windows")    
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
      "Choose the type of build, recommended options are: Debug or Release")
endif()
# Hide variable to MSVC users, since it is not needed
if(MSVC)
   mark_as_advanced(CMAKE_BUILD_TYPE)
endif()

######################

### this makes everything go in $ROBOTICSLAB_KINEMATICS_DYNAMICS_DIR/lib and $ROBOTICSLAB_KINEMATICS_DYNAMICS_DIR/bin
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
message(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")
mark_as_advanced(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)
##########################################

# Pick up our cmake modules - they are all in the cmake subdirectory
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${ROBOTICSLAB_KINEMATICS_DYNAMICS_MODULE_PATH})

set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS CACHE INTERNAL "appended link dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES CACHE INTERNAL "appended libraries")

# Generate roboticslab-kinematics-dynamics-export.h
include(GenerateExportHeader)
file(WRITE ${CMAKE_BINARY_DIR}/generated_src/empty.c "")
add_library(ROBOTICSLAB_KINEMATICS_DYNAMICS ${CMAKE_BINARY_DIR}/generated_src/empty.c)
generate_export_header(ROBOTICSLAB_KINEMATICS_DYNAMICS
                       EXPORT_FILE_NAME ${CMAKE_BINARY_DIR}/generated_include/roboticslab-kinematics-dynamics-export.h)
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/generated_include CACHE INTERNAL "appended header dirs")
install(FILES ${CMAKE_BINARY_DIR}/generated_include/roboticslab-kinematics-dynamics-export.h
        DESTINATION include)

if(ENABLE_coverage)
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage") # enabling coverage
  endif()
endif()

# Bootstrap YCM
# https://github.com/robotology/ycm/issues/118
set(YCM_TAG v0.2.2)
include(YCMBootstrap)

# Superbuild phase - include color-debug
include(FindOrBuildPackage)
find_or_build_package(COLOR_DEBUG)

if(USE_SYSTEM_COLOR_DEBUG)
  find_package(COLOR_DEBUG QUIET)
  if(COLOR_DEBUG_FOUND)
    include_directories(${COLOR_DEBUG_INCLUDE_DIRS})
  endif()
endif()

# add main contents
add_subdirectory(share)
add_subdirectory(libraries)
add_subdirectory(programs)
if(ENABLE_tests)
    add_subdirectory(test)
endif()

# export our variables to a TEOConfig.cmake creation
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS} ${LIBRARY_OUTPUT_PATH})
configure_file(${CMAKE_SOURCE_DIR}/cmake/template/ROBOTICSLAB_KINEMATICS_DYNAMICSConfig.cmake.in
               ${CMAKE_BINARY_DIR}/ROBOTICSLAB_KINEMATICS_DYNAMICSConfig.cmake @ONLY)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/template/ROBOTICSLAB_KINEMATICS_DYNAMICSConfigUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ROBOTICSLAB_KINEMATICS_DYNAMICSConfigUninstall.cmake" IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/ROBOTICSLAB_KINEMATICS_DYNAMICSConfigUninstall.cmake)
