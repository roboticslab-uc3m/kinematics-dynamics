option(ENABLE_tests "Choose if you want to compile tests" ON)

if(ENABLE_tests)

add_subdirectory(gtest-1.7.0)
enable_testing()

find_package(YARP REQUIRED)

get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

include_directories(${gtest_SOURCE_DIR}/include
                    ${gtest_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${_common_includes})

add_executable(testKdlSolver testKdlSolver.cpp)
add_dependencies(testKdlSolver COLOR_DEBUG)
target_link_libraries(testKdlSolver YARP::YARP_OS YARP::YARP_dev gtest_main)

add_executable(testKdlSolverFromFile testKdlSolverFromFile.cpp)
add_dependencies(testKdlSolverFromFile COLOR_DEBUG)
target_link_libraries(testKdlSolverFromFile YARP::YARP_OS YARP::YARP_dev gtest_main)

add_executable(testAsibotSolverFromFile testAsibotSolverFromFile.cpp)
add_dependencies(testAsibotSolverFromFile COLOR_DEBUG)
target_link_libraries(testAsibotSolverFromFile YARP::YARP_OS YARP::YARP_dev gtest_main)

add_executable(testBasicCartesianControl testBasicCartesianControl.cpp)
add_dependencies(testBasicCartesianControl COLOR_DEBUG)
target_link_libraries(testBasicCartesianControl YARP::YARP_OS YARP::YARP_dev gtest_main)

endif()
