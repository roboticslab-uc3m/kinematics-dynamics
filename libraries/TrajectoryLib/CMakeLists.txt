cmake_dependent_option(ENABLE_TrajectoryLib "Enable/disable TrajectoryLib library" ON
                       "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND" OFF)

unset(ENABLE_TrajectoryLib)

if(NOT DEFINED ENABLE_TrajectoryLib)
    set(ENABLE_TrajectoryLib ON CACHE BOOL "")
endif()

if(NOT orocos_kdl_FOUND)
    if(ENABLE_TrajectoryLib)
        message(WARNING "orocos_kdl package not found, disabling TrajectoryLib")
    endif()
    if(NOT DEFINED ENABLE_TrajectoryLib-STORED)
        set(ENABLE_TrajectoryLib-STORED "${ENABLE_TrajectoryLib}" CACHE INTERNAL "Stored value of ENABLE_TrajectoryLib prior to forcing OFF")
    endif()
    set(ENABLE_TrajectoryLib OFF CACHE BOOL "Enable/disable TrajectoryLib" FORCE)
elseif(DEFINED ENABLE_TrajectoryLib-STORED AND NOT ENABLE_TrajectoryLib)
    set(ENABLE_TrajectoryLib "${ENABLE_TrajectoryLib-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_TrajectoryLib-STORED CACHE)
endif()

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

endif()
