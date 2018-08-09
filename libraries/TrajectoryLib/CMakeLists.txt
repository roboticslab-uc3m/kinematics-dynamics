verbose_dependent_option(TrajectoryLib DEPENDS_WARNING orocos_kdl_FOUND
                                       DEPENDS_OTHER ENABLE_KdlVectorConverterLib)

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
