if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)

    enable_testing()

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

    include_directories(${GTestSources_INCLUDE_DIR}})

    # testKinRepresentation

    if(TARGET KinematicRepresentationLib)
        add_executable(testKinRepresentation testKinRepresentation.cpp)

        target_link_libraries(testKinRepresentation KinematicRepresentationLib
                                                    gtest_main)

        gtest_add_tests(testKinRepresentation "" AUTO)
    endif()

    # testScrewTheory

    if(TARGET ScrewTheoryLib)
        add_executable(testScrewTheory testScrewTheory.cpp)

        target_link_libraries(testScrewTheory ScrewTheoryLib
                                              gtest_main)

        gtest_add_tests(testScrewTheory "" AUTO)
    endif()

    # testKdlSolver

    add_executable(testKdlSolver testKdlSolver.cpp)

    target_link_libraries(testKdlSolver YARP::YARP_OS
                                        YARP::YARP_dev
                                        ROBOTICSLAB::ColorDebug
                                        KinematicsDynamicsInterfaces
                                        gtest_main)

    gtest_add_tests(testKdlSolver "" AUTO)

    # testKdlSolverFromFile

    add_executable(testKdlSolverFromFile testKdlSolverFromFile.cpp)

    target_link_libraries(testKdlSolverFromFile YARP::YARP_OS
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug
                                                KinematicsDynamicsInterfaces
                                                gtest_main)

    gtest_add_tests(testKdlSolverFromFile "" AUTO)

    # testAsibotSolverFromFile

    if(TARGET KinematicRepresentationLib)
        add_executable(testAsibotSolverFromFile testAsibotSolverFromFile.cpp)

        target_link_libraries(testAsibotSolverFromFile YARP::YARP_OS
                                                       YARP::YARP_dev
                                                       ROBOTICSLAB::ColorDebug
                                                       KinematicRepresentationLib
                                                       KinematicsDynamicsInterfaces
                                                       gtest_main)

        gtest_add_tests(testAsibotSolverFromFile "" AUTO)
    endif()

    # testKdlTrajectory

    if(TARGET TrajectoryLib)
        add_executable(testKdlTrajectory testKdlTrajectory.cpp)

        target_link_libraries(testKdlTrajectory TrajectoryLib
                                                gtest_main)

        gtest_add_tests(testKdlTrajectory "" AUTO)
    endif()

    # testBasicCartesianControl

    add_executable(testBasicCartesianControl testBasicCartesianControl.cpp)

    target_link_libraries(testBasicCartesianControl YARP::YARP_OS
                                                    YARP::YARP_dev
                                                    ROBOTICSLAB::ColorDebug
                                                    KinematicsDynamicsInterfaces
                                                    gtest_main)

    gtest_add_tests(testBasicCartesianControl "" AUTO)

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()
