cmake_dependent_option(ENABLE_TrajectoryLib "Enable/disable TrajectoryLib library" ON
                       ENABLE_KdlVectorConverterLib OFF)

if(ENABLE_TrajectoryLib)

find_package(orocos_kdl REQUIRED)

set(_trayectory_lib_sources ICartesianTrajectory.hpp
                            ITrajectory.hpp
                            KdlTrajectory.cpp
                            KdlTrajectory.hpp)

# Set up our library.
add_library(TrajectoryLib ${_trayectory_lib_sources})

add_dependencies(TrajectoryLib COLOR_DEBUG)

set_property(TARGET TrajectoryLib PROPERTY PUBLIC_HEADER ICartesianTrajectory.hpp
                                                         ITrajectory.hpp
                                                         KdlTrajectory.hpp)

target_link_libraries(TrajectoryLib PUBLIC ${orocos_kdl_LIBRARIES}
                                    PRIVATE KdlVectorConverterLib)

target_include_directories(TrajectoryLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS TrajectoryLib
        EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS TrajectoryLib)

endif()
