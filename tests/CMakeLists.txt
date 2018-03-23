find_package(GTestSources 1.6.0 QUIET)

if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

    include_directories(${GTestSources_INCLUDE_DIR}
                        ${YARP_INCLUDE_DIRS})

    # testKinRepresentation

    add_executable(testKinRepresentation testKinRepresentation.cpp)

    add_dependencies(testKinRepresentation COLOR_DEBUG)

    target_link_libraries(testKinRepresentation KinematicRepresentationLib
                                                gtest_main)

    # testKdlSolver

    add_executable(testKdlSolver testKdlSolver.cpp)

    add_dependencies(testKdlSolver COLOR_DEBUG)

    target_link_libraries(testKdlSolver YARP::YARP_OS
                                        YARP::YARP_dev
                                        KinematicsDynamicsInterfaces
                                        gtest_main)

    # testKdlSolverFromFile

    add_executable(testKdlSolverFromFile testKdlSolverFromFile.cpp)

    add_dependencies(testKdlSolverFromFile COLOR_DEBUG)

    target_link_libraries(testKdlSolverFromFile YARP::YARP_OS
                                                YARP::YARP_dev
                                                KinematicsDynamicsInterfaces
                                                gtest_main)

    # testAsibotSolverFromFile

    add_executable(testAsibotSolverFromFile testAsibotSolverFromFile.cpp)

    add_dependencies(testAsibotSolverFromFile COLOR_DEBUG)

    target_link_libraries(testAsibotSolverFromFile YARP::YARP_OS
                                                   YARP::YARP_dev
                                                   KinematicRepresentationLib
                                                   KinematicsDynamicsInterfaces
                                                   gtest_main)

    # testKdlTrajectory

    add_executable(testKdlTrajectory testKdlTrajectory.cpp)

    add_dependencies(testKdlTrajectory COLOR_DEBUG)

    target_link_libraries(testKdlTrajectory TrajectoryLib
                                            gtest_main)

    # testBasicCartesianControl

    add_executable(testBasicCartesianControl testBasicCartesianControl.cpp)

    add_dependencies(testBasicCartesianControl COLOR_DEBUG)

    target_link_libraries(testBasicCartesianControl YARP::YARP_OS
                                                    YARP::YARP_dev
                                                    KinematicsDynamicsInterfaces
                                                    gtest_main)

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()
