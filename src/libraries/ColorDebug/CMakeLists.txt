# Copyright: UC3M 2014 (C)
# Author: Juan G. Victores @ http://roboticslab.uc3m.es/roboticslab/people/jg-victores

# reduce warning level with cmake 2.6
cmake_minimum_required(VERSION 2.6)
#cmake policies
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(ColorDebug)

set(COLOR_DEBUG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")

# install target
install (FILES "${CMAKE_CURRENT_SOURCE_DIR}/ColorDebug.hpp"
         DESTINATION include)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/template/ColorDebugConfigUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ColorDebugConfigUninstall.cmake" IMMEDIATE @ONLY)

if(NOT COLOR_DEBUG_PART_OF_PROJECT)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/ColorDebugConfigUninstall.cmake)
endif(NOT COLOR_DEBUG_PART_OF_PROJECT)

