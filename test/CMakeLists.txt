option(ENABLE_tests "Choose if you want to compile tests" ON)

if(ENABLE_tests)

find_path(GTEST_SOURCE_DIR src/gtest.cc
                           HINTS $ENV{GTEST_ROOT}
                                 ${GTEST_ROOT}
                           PATHS /usr/src/gtest)

find_path(GTEST_INCLUDE_DIR gtest/gtest.h
                            HINTS $ENV{GTEST_ROOT}/include
                                  ${GTEST_ROOT}/include)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTest DEFAULT_MSG GTEST_SOURCE_DIR GTEST_INCLUDE_DIR)
mark_as_advanced(GTEST_SOURCE_DIR GTEST_INCLUDE_DIR)

if(NOT GTest_FOUND)
    message(SEND_ERROR "GTest not found")
endif()

add_subdirectory(${GTEST_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

find_package(YARP REQUIRED)

get_property(_common_includes GLOBAL PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS)

include_directories(${GTEST_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${_common_includes})

add_executable(testKinRepresentation testKinRepresentation.cpp)
add_dependencies(testKinRepresentation COLOR_DEBUG)
target_link_libraries(testKinRepresentation gtest_main KinematicRepresentationLib)

add_executable(testKdlSolver testKdlSolver.cpp)
add_dependencies(testKdlSolver COLOR_DEBUG)
target_link_libraries(testKdlSolver YARP::YARP_OS YARP::YARP_dev gtest_main)

add_executable(testKdlSolverFromFile testKdlSolverFromFile.cpp)
add_dependencies(testKdlSolverFromFile COLOR_DEBUG)
target_link_libraries(testKdlSolverFromFile YARP::YARP_OS YARP::YARP_dev gtest_main)

add_executable(testAsibotSolverFromFile testAsibotSolverFromFile.cpp)
add_dependencies(testAsibotSolverFromFile COLOR_DEBUG)
target_link_libraries(testAsibotSolverFromFile YARP::YARP_OS YARP::YARP_dev gtest_main KinematicRepresentationLib)

add_executable(testKdlTrajectory testKdlTrajectory.cpp)
add_dependencies(testKdlTrajectory COLOR_DEBUG)
target_link_libraries(testKdlTrajectory YARP::YARP_OS YARP::YARP_dev gtest_main TrajectoryLib)

add_executable(testBasicCartesianControl testBasicCartesianControl.cpp)
add_dependencies(testBasicCartesianControl COLOR_DEBUG)
target_link_libraries(testBasicCartesianControl YARP::YARP_OS YARP::YARP_dev gtest_main)

endif()
