# Testing things #########################################################################################
add_subdirectory(gtest-1.7.0)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${TEO_INCLUDE_DIRS})
link_directories(${TEO_LINK_DIRS})

add_executable(testKdlController testKdlController.cpp)
target_link_libraries(testKdlController ${TEO_LIBRARIES} gtest gtest_main)

