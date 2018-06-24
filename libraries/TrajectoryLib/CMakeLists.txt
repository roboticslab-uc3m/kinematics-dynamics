if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_TrajectoryLib OR ENABLE_TrajectoryLib))
    message(WARNING "orocos_kdl package not found, disabling TrajectoryLib")
endif()

cmake_dependent_option(ENABLE_TrajectoryLib "Enable/disable TrajectoryLib library" ON
                       "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND" OFF)

if(ENABLE_TrajectoryLib)

    add_library(TrajectoryLib SHARED ITrajectory.hpp
                                     ICartesianTrajectory.hpp
                                     KdlTrajectory.cpp
                                     KdlTrajectory.hpp)

    set_property(TARGET TrajectoryLib PROPERTY PUBLIC_HEADER ICartesianTrajectory.hpp
                                                             ITrajectory.hpp
                                                             KdlTrajectory.hpp)

    target_link_libraries(TrajectoryLib PUBLIC ${orocos_kdl_LIBRARIES}
                                        PRIVATE KdlVectorConverterLib
                                                ROBOTICSLAB::ColorDebug)

    target_include_directories(TrajectoryLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS TrajectoryLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

else()

    set(ENABLE_TrajectoryLib OFF CACHE BOOL "Enable/disable TrajectoryLib library" FORCE)

endif()
