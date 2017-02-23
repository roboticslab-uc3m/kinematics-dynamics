# Testing things #########################################################################################
add_subdirectory(gtest-1.7.0)
enable_testing()

find_package(YARP REQUIRED)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${TEO_INCLUDE_DIRS})
link_directories(${TEO_LINK_DIRS})

add_executable(testKdlSolver testKdlSolver.cpp)
target_link_libraries(testKdlSolver ${YARP_LIBRARIES} gtest gtest_main)

add_executable(testKdlSolverFromFile testKdlSolverFromFile.cpp)
target_link_libraries(testKdlSolverFromFile ${YARP_LIBRARIES} gtest gtest_main)

add_executable(testBasicCartesianControl testBasicCartesianControl.cpp)
target_link_libraries(testBasicCartesianControl ${YARP_LIBRARIES} gtest gtest_main)

