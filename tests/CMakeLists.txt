if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)

    enable_testing()

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

    include_directories(${GTestSources_INCLUDE_DIR})

    # testKinRepresentation

    if(ENABLE_KinematicRepresentationLib)
        add_executable(testKinRepresentation testKinRepresentation.cpp)

        target_link_libraries(testKinRepresentation ROBOTICSLAB::KinematicRepresentationLib
                                                    gtest_main)

        gtest_discover_tests(testKinRepresentation)
    endif()

    # testScrewTheory

    if(ENABLE_ScrewTheoryLib)
        add_executable(testScrewTheory testScrewTheory.cpp)

        target_link_libraries(testScrewTheory ROBOTICSLAB::ScrewTheoryLib
                                              gtest_main)

        gtest_discover_tests(testScrewTheory)
    endif()

    # testKdlSolver

    add_executable(testKdlSolver testKdlSolver.cpp)

    target_link_libraries(testKdlSolver YARP::YARP_os
                                        YARP::YARP_dev
                                        ROBOTICSLAB::ColorDebug
                                        ROBOTICSLAB::KinematicsDynamicsInterfaces
                                        gtest_main)

    gtest_discover_tests(testKdlSolver)

    # testKdlSolverFromFile

    add_executable(testKdlSolverFromFile testKdlSolverFromFile.cpp)

    target_link_libraries(testKdlSolverFromFile YARP::YARP_os
                                                YARP::YARP_dev
                                                ROBOTICSLAB::ColorDebug
                                                ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                gtest_main)

    gtest_discover_tests(testKdlSolverFromFile)

    # testAsibotSolverFromFile

    if(ENABLE_KinematicRepresentationLib)
        add_executable(testAsibotSolverFromFile testAsibotSolverFromFile.cpp)

        target_link_libraries(testAsibotSolverFromFile YARP::YARP_os
                                                       YARP::YARP_dev
                                                       ROBOTICSLAB::ColorDebug
                                                       ROBOTICSLAB::KinematicRepresentationLib
                                                       ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                       gtest_main)

        gtest_discover_tests(testAsibotSolverFromFile)
    endif()

    # testKdlTrajectory

    if(ENABLE_TrajectoryLib)
        add_executable(testKdlTrajectory testKdlTrajectory.cpp)

        target_link_libraries(testKdlTrajectory ROBOTICSLAB::TrajectoryLib
                                                gtest_main)

        gtest_discover_tests(testKdlTrajectory)
    endif()

    # testBasicCartesianControl

    if(ROBOTICSLAB_YARP_DEVICES_FOUND)
        add_executable(testBasicCartesianControl testBasicCartesianControl.cpp)

        target_link_libraries(testBasicCartesianControl YARP::YARP_os
                                                        YARP::YARP_dev
                                                        ROBOTICSLAB::ColorDebug
                                                        ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                        gtest_main)

        gtest_discover_tests(testBasicCartesianControl)
    endif()

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()
