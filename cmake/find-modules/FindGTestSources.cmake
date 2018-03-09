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
# Tested with the Ubuntu package `libgtest-dev` and the googletest
# repository hosted on GitHub and cloned to the local machine.
#
# Supported versions: v1.6, v1.7, v1.8.
#
# Also, this module adds the following macros:
#
# * gtest_add_tests (as in FindGTest.cmake)

find_package(GTest QUIET)

if(NOT GTestSources_SOURCE_DIR)
    find_path(GTestSources_SOURCE_DIR src/gtest.cc
                                      HINTS $ENV{GTEST_ROOT}
                                            ${GTEST_ROOT}
                                      PATHS /usr/src/gtest
                                            /usr/src/googletest
                                      PATH_SUFFIXES googletest)
endif()

if(NOT GTestSources_INCLUDE_DIR)
    # Look for local headers *before* /usr/include (hence the NO_X_PATH params)
    find_path(GTestSources_INCLUDE_DIR gtest/gtest.h
                                       HINTS ${GTestSources_SOURCE_DIR}
                                             $ENV{GTEST_ROOT}
                                             ${GTEST_ROOT}
                                       PATH_SUFFIXES include
                                       NO_CMAKE_PATH
                                       NO_CMAKE_ENVIRONMENT_PATH)
endif()

set(GTestSources_VERSION GTestSources_VERSION-NOT_FOUND)

if(GTestSources_SOURCE_DIR AND GTestSources_INCLUDE_DIR)
    set(_cmake_include_dirs ${CMAKE_REQUIRED_INCLUDES})

    include(CheckCXXSourceCompiles)
    list(APPEND CMAKE_REQUIRED_INCLUDES ${GTestSources_INCLUDE_DIR})

    check_cxx_source_compiles("
            #include <gtest/gtest.h>
            int main() {
                typedef const char* (testing::TestInfo::*fun)() const;
                fun f = &testing::TestInfo::type_param;
                return 0;
            }"
        _gtest_compatible_1_6_0)

    check_cxx_source_compiles("
            #include <gtest/gtest.h>
            int main() {
                typedef bool (testing::TestInfo::*fun)() const;
                fun f = &testing::TestInfo::is_reportable;
                return 0;
            }"
        _gtest_compatible_1_7_0)

    check_cxx_source_compiles("
            #include <gtest/gtest.h>
            int main() {
                typedef const char* (testing::TestInfo::*fun)() const;
                fun f = &testing::TestInfo::file;
                return 0;
            }"
        _gtest_compatible_1_8_0)

    if(_gtest_compatible_1_8_0)
        set(GTestSources_VERSION 1.8.0)
    elseif(_gtest_compatible_1_7_0)
        set(GTestSources_VERSION 1.7.0)
    elseif(_gtest_compatible_1_6_0)
        set(GTestSources_VERSION 1.6.0)
    else()
        message(STATUS "FindGTestSources.cmake reports unhandled GTest version (<1.6.0)")
    endif()

    set(CMAKE_REQUIRED_INCLUDES "${_cmake_include_dirs}")
    unset(_cmake_include_dirs)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTestSources FOUND_VAR GTestSources_FOUND
                                               REQUIRED_VARS GTestSources_SOURCE_DIR
                                                             GTestSources_INCLUDE_DIR
                                               VERSION_VAR GTestSources_VERSION)

mark_as_advanced(GTestSources_SOURCE_DIR
                 GTestSources_INCLUDE_DIR)
