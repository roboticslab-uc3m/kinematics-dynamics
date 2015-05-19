# Copyright: Universidad Carlos III de Madrid (C) 2013;
# Authors: Juan G. Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Exploit new cmake 2.6 features (export). 

# reduce warning level with cmake 2.6
cmake_minimum_required(VERSION 2.6)
#cmake policies
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(TEO)

### options: cpp libraries
option(ENABLE_TeoYarp "Choose if you want to compile TeoYarp" TRUE)

### options: cpp modules
option(ENABLE_teoCartesianServer "Choose if you want to compile teoCartesianServer" FALSE)  #-- teoCartesianServer OFF for now
option(ENABLE_teoGravityCompensator "Choose if you want to compile teoGravityCompensator" TRUE)
option(ENABLE_teoSim "Choose if you want to compile teoSim" TRUE)
option(ENABLE_testTeoYarp "Choose if you want to compile testTeoYarp" TRUE)

### options: force default
#-- kdlcontroller OFF for now
option(ENABLE_TeoYarp_kdlsolver "Enable/disable compilation of TeoYarp_kdlsolver" TRUE)
option(ENABLE_TeoYarp_ravepart "Enable/disable compilation of TeoYarp_ravepart" TRUE)

##### test and coverage options
option(ENABLE_tests "Choose if you want to compile tests" TRUE)
option(ENABLE_coverage "Choose if you want to enable coverage collection" FALSE)

# ColorDebug options
option(ColorDebug_FULL_FILE "Choose if you want to compile with CD_FULL_FILE" FALSE)
if(ColorDebug_FULL_FILE)
    add_definitions(-DCD_FULL_FILE)
endif(ColorDebug_FULL_FILE)

option(ColorDebug_HIDE_ERROR "Choose if you want to compile with CD_HIDE_ERROR" FALSE)
if(ColorDebug_HIDE_ERROR)
    add_definitions(-DCD_HIDE_ERROR)
endif(ColorDebug_HIDE_ERROR)

option(ColorDebug_HIDE_WARNING "Choose if you want to compile with CD_HIDE_WARNING" FALSE)
if(ColorDebug_HIDE_WARNING)
    add_definitions(-DCD_HIDE_WARNING)
endif(ColorDebug_HIDE_WARNING)

option(ColorDebug_HIDE_SUCCESS "Choose if you want to compile with CD_HIDE_SUCCESS" FALSE)
if(ColorDebug_HIDE_SUCCESS)
    add_definitions(-DCD_HIDE_SUCCESS)
endif(ColorDebug_HIDE_SUCCESS)

option(ColorDebug_HIDE_INFO "Choose if you want to compile with CD_HIDE_INFO" FALSE)
if(ColorDebug_HIDE_INFO)
    add_definitions(-DCD_HIDE_INFO)
endif(ColorDebug_HIDE_INFO)

option(ColorDebug_HIDE_DEBUG "Choose if you want to compile with CD_HIDE_DEBUG" FALSE)
if(ColorDebug_HIDE_DEBUG)
    add_definitions(-DCD_HIDE_DEBUG)
endif(ColorDebug_HIDE_DEBUG)

if(MSVC)
  MESSAGE(STATUS "Running on windows")    
  set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
    "Choose the type of build, recommanded options are: Debug or Release")
endif(NOT CMAKE_BUILD_TYPE)
# Hide variable to MSVC users, since it is not needed
if (MSVC)
   mark_as_advanced(CMAKE_BUILD_TYPE)
endif(MSVC)

######################

### this makes everything go in $TEO_DIR/lib and $TEO_DIR/bin
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
message(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")
# this doesn't happen automatically for makefiles
make_directory(${LIBRARY_OUTPUT_PATH})
make_directory(${EXECUTABLE_OUTPUT_PATH})
# and let us clean their contents on a "make clean"
##set_directory_properties(PROPERTIES LIBRARY_OUTPUT_PATH ADDITIONAL_MAKE_CLEAN_FILES)
##set_directory_properties(PROPERTIES EXECUTABLE_OUTPUT_PATH ADDITIONAL_MAKE_CLEAN_FILES)
mark_as_advanced(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)
mark_as_advanced(CMAKE_BACKWARDS_COMPATIBILITY)
##########################################

# Pick up our cmake modules - they are all in the cmake subdirectory
set(TEO_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${TEO_MODULE_PATH})

#set_property(GLOBAL PROPERTY TEO_INCLUDE_DIRS)
#set_property(GLOBAL PROPERTY TEO_LINK_DIRS)
#set_property(GLOBAL PROPERTY TEO_LIBRARIES)
#set_property(GLOBAL PROPERTY TEO_TARGETS)
set(TEO_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(TEO_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(TEO_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)

if(ENABLE_coverage)
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage") # enabling coverage
  endif(CMAKE_COMPILER_IS_GNUCXX)
endif(ENABLE_coverage)

# add main contents
add_subdirectory(share)
add_subdirectory(libraries)
add_subdirectory(programs)
if (ENABLE_tests)
    add_subdirectory(test)
endif (ENABLE_tests)

# export our variables to a TEOConfig.cmake creation
set(TEO_LINK_DIRS ${TEO_LINK_DIRS} ${LIBRARY_OUTPUT_PATH})
configure_file(${CMAKE_SOURCE_DIR}/cmake/template/TEOConfig.cmake.in
               ${CMAKE_BINARY_DIR}/TEOConfig.cmake @ONLY)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/template/TEOConfigUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/TEOConfigUninstall.cmake" IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/TEOConfigUninstall.cmake)

