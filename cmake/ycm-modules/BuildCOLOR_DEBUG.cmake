include(YCMEPHelper)

ycm_ep_helper(COLOR_DEBUG TYPE GIT
              STYLE GITHUB
              REPOSITORY roboticslab-uc3m/color-debug.git
              TAG master)

# Include path to ColorDebug.hpp.
ExternalProject_Get_Property(COLOR_DEBUG INSTALL_DIR)
include_directories(${INSTALL_DIR}/${CMAKE_INSTALL_INCLUDEDIR})

# CMake has not downloaded color-debug yet (this happens on build step).
if(NOT COLOR_DEBUG_FOUND)
    message(STATUS "Build COLOR_DEBUG target and configure project again to make advanced CD options available on UI.")
    # Fails on first configure due to the lack of a cache file.
    #execute_process(COMMAND ${CMAKE_COMMAND} --build . --target COLOR_DEBUG WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
else()
    # Load COLOR_DEBUGConfig.cmake, which in turn includes ColorDebugOptions.cmake.
    find_package(COLOR_DEBUG QUIET)
endif()
