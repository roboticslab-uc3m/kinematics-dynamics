if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_ScrewTheoryIkSolverLib OR ENABLE_ScrewTheoryIkSolverLib))
    message(WARNING "orocos_kdl package not found, disabling ScrewTheoryIkSolverLib")
endif()

cmake_dependent_option(ENABLE_ScrewTheoryIkSolverLib "Enable/disable ScrewTheoryIkSolverLib library" ON
                       orocos_kdl_FOUND OFF)

if(ENABLE_ScrewTheoryIkSolverLib)

    # Set up our library.
    add_library(ScrewTheoryIkSolverLib SHARED ScrewTheoryIkSolver.cpp
                                              ScrewTheoryIkSolver.hpp)

    set_target_properties(ScrewTheoryIkSolverLib PROPERTIES PUBLIC_HEADER ScrewTheoryIkSolver.hpp)

    target_link_libraries(ScrewTheoryIkSolverLib PUBLIC ${orocos_kdl_LIBRARIES}
                                                 PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(ScrewTheoryIkSolverLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS ScrewTheoryIkSolverLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

else()

    set(ENABLE_ScrewTheoryIkSolverLib OFF CACHE BOOL "Enable/disable ScrewTheoryIkSolverLib library" FORCE)

endif()
