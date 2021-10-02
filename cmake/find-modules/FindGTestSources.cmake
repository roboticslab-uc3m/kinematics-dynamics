# Find the GTest headers and sources.
#
# Sets the following variables:
#
# * GTestSources_FOUND       - system has GTest sources
# * GTestSources_SOURCE_DIR  - GTest source dir (with CMakeLists.txt)
# * GTestSources_INCLUDE_DIR - GTest include directory (public headers)
# * GTestSources_VERSION     - GTest version (if supported)
#
# You can set the GTEST_ROOT environment variable to be used as a
# hint by FindGTestSources to locate googletest source directory.
#
# Tested with the Ubuntu package `googletest` and the googletest
# repository hosted on GitHub and cloned to the local machine.
#
# Supported versions: 1.8+.
#
# Also, this module adds the following macros:
#
# * gtest_add_tests (as in FindGTest.cmake)
# * gtest_discover_tests (as in FindGTest.cmake)

find_package(GTest QUIET)

find_path(GTestSources_SOURCE_DIR googletest/src/gtest.cc
                                  HINTS $ENV{GTEST_ROOT}
                                        ${GTEST_ROOT}
                                  PATHS /usr/src/googletest)

find_path(GTestSources_INCLUDE_DIR gtest/gtest.h
                                   HINTS ${GTestSources_SOURCE_DIR}
                                         $ENV{GTEST_ROOT}
                                         ${GTEST_ROOT}
                                   PATH_SUFFIXES googletest/include)

set(GTestSources_VERSION GTestSources_VERSION-NOT_FOUND)

if(GTestSources_SOURCE_DIR AND GTestSources_INCLUDE_DIR AND EXISTS ${GTestSources_SOURCE_DIR}/CMakeLists.txt)
    file(READ "${GTestSources_SOURCE_DIR}/CMakeLists.txt" _cmakelists)
    string(REGEX MATCH "GOOGLETEST_VERSION ([0-9]+\\.[0-9]+(\\.[0-9]+)?)" _match "${_cmakelists}")

    if(NOT "${CMAKE_MATCH_1}" STREQUAL "")
        set(GTestSources_VERSION "${CMAKE_MATCH_1}")
    elseif(EXISTS ${GTestSources_SOURCE_DIR}/googlemock/CMakeLists.txt)
        set(GTestSources_VERSION 1.8.0)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTestSources FOUND_VAR GTestSources_FOUND
                                               REQUIRED_VARS GTestSources_SOURCE_DIR
                                                             GTestSources_INCLUDE_DIR
                                               VERSION_VAR GTestSources_VERSION)

if(GTestSources_FOUND)
    # should be non-cache variables, but CMP0077 strikes in
    set(BUILD_GMOCK OFF CACHE BOOL "")
    set(BUILD_GTEST ON CACHE BOOL "")
    set(INSTALL_GTEST OFF CACHE BOOL "")
    mark_as_advanced(BUILD_GMOCK BUILD_GTEST INSTALL_GTEST)
endif()

mark_as_advanced(GTestSources_SOURCE_DIR GTestSources_INCLUDE_DIR)
