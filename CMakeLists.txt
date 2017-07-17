# Copyright: Universidad Carlos III de Madrid (C) 2017;
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)

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

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Pick up our cmake modules - they are all in the cmake subdirectory.
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_MODULE_DIR ${CMAKE_SOURCE_DIR}/cmake)

# Let cmake use them.
list(APPEND CMAKE_MODULE_PATH ${ROBOTICSLAB_KINEMATICS_DYNAMICS_MODULE_DIR})

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
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(ENABLE_coverage)
    # List supported compilers.
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

# Store common include directories in global scope.
set_property(GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/generated_include)

# Store list of exported targets in global scope.
set_property(GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS)

# add main contents
add_subdirectory(share)
add_subdirectory(libraries)
add_subdirectory(programs)
if(ENABLE_tests)
    add_subdirectory(test)
endif()

# Store the package in the user registry.
export(PACKAGE ${PROJECT_NAME})

# Create and install config and version files.
include(InstallBasicPackageFiles)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)
get_property(_exported_targets GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS)

# Set build/install pairs of paths for each exported property.
# Must end in _DIR instead of _DIRS as the former is a default PATH_VARS suffix
# recognized by install_basic_package_files(). Alternatively, use EXTRA_PATH_VARS_SUFFIX.
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_BUILD_INCLUDE_DIR ${_common_includes})
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})

install_basic_package_files(${PROJECT_NAME}
                            VERSION 0.1.0
                            COMPATIBILITY AnyNewerVersion
                            TARGETS_PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            NAMESPACE ROBOTICSLAB::)

# Configure and create uninstall target.
include(AddUninstallTarget)
