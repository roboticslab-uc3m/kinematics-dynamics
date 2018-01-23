# Find the GTest headers and sources.
#
# Sets the following variables:
#
# GTestSources_FOUND       - system has GTest sources
# GTestSources_SOURCE_DIR  - GTest source dir (with CMakeLists.txt)
# GTestSources_INCLUDE_DIR - GTest include directory (public headers)
#
# You can set the GTEST_ROOT environment variable to be used as a
# hint by FindGTestSources to locate googletest source directory.
#
# Tested with the Ubuntu package `libgtest-dev` and the googletest
# repository hosted on GitHub and cloned to the local machine.
#
# Supported versions: v1.6, v1.7.

if(NOT GTestSources_SOURCE_DIR)
    find_path(GTestSources_SOURCE_DIR src/gtest.cc
                                      HINTS $ENV{GTEST_ROOT}
                                            ${GTEST_ROOT}
                                      PATHS /usr/src/gtest)
endif()

if(NOT GTestSources_INCLUDE_DIR)
    find_path(GTestSources_INCLUDE_DIR gtest/gtest.h
                                       HINTS $ENV{GTEST_ROOT}/include
                                             ${GTEST_ROOT}/include)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTestSources REQUIRED_VARS GTestSources_SOURCE_DIR
                                                             GTestSources_INCLUDE_DIR)

mark_as_advanced(GTestSources_SOURCE_DIR
                 GTestSources_INCLUDE_DIR)
