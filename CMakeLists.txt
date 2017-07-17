# Copyright: Universidad Carlos III de Madrid (C) 2017;
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(ROBOTICSLAB_KINEMATICS_DYNAMICS)

# Force building shared libraries and YARP DL modules.
set(BUILD_SHARED_LIBS TRUE CACHE INTERNAL "hide this!")

# Add suffix for debug libraries.
if(MSVC)
    message(STATUS "Running on windows")    
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
    # Possible values.
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    if(NOT CMAKE_BUILD_TYPE)
        # Encourage the user to specify build type.
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Pick up our cmake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Generate export header.
include(GenerateExportHeader)

# Dummy source file and library target for generate_export_header().
file(WRITE ${CMAKE_BINARY_DIR}/generated_src/empty.c "")
add_library(ROBOTICSLAB_KINEMATICS_DYNAMICS ${CMAKE_BINARY_DIR}/generated_src/empty.c)

generate_export_header(ROBOTICSLAB_KINEMATICS_DYNAMICS
                       EXPORT_FILE_NAME ${CMAKE_BINARY_DIR}/generated_include/roboticslab-kinematics-dynamics-export.h)

install(FILES ${CMAKE_BINARY_DIR}/generated_include/roboticslab-kinematics-dynamics-export.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Enabling coverage.
option(ENABLE_coverage "Choose if you want to enable coverage collection" OFF)

if(ENABLE_coverage)
    # List supported compilers.
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    endif()
endif()

# Bootstrap YCM.
# https://github.com/robotology/ycm/issues/118
set(YCM_TAG v0.2.2)
include(YCMBootstrap)

# Superbuild phase - include color-debug.
include(FindOrBuildPackage)
find_or_build_package(COLOR_DEBUG)

# find_or_build_package() doesn't bring into scope COLOR_DEBUG_INCLUDE_DIRS
# nor other config variables. If a system copy is found, we have to call the
# find_package() command and proceed as usual.
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

# Add main contents.
add_subdirectory(share)
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(test)

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
