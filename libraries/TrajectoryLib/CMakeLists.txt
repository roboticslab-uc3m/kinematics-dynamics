cmake_dependent_option(ENABLE_Gait "Enable/disable TrajectoryLib library" ON
                       ENABLE_KdlVectorConverterLib OFF)

if(ENABLE_TrajectoryLib)

find_package(orocos_kdl REQUIRED)

set(_trayectory_lib_sources Trajectory.hpp
                            LineTrajectory.cpp
                            LineTrajectory.hpp)

if(HAVE_GAIT AND ENABLE_Gait)
    list(APPEND _trayectory_lib_sources GaitTrajectory.cpp
                                        GaitTrajectory.hpp)
endif()

# Set up our library.
add_library(TrajectoryLib ${_trayectory_lib_sources})

set_property(TARGET KdlVectorConverterLib PROPERTY PUBLIC_HEADER Trajectory.hpp
                                                                 LineTrajectory.hpp)

target_link_libraries(TrajectoryLib PUBLIC ${orocos_kdl_LIBRARIES}
                                           KdlVectorConverterLib)

target_include_directories(KdlVectorConverterLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(HAVE_GAIT AND ENABLE_Gait)
    set_property(TARGET KdlVectorConverterLib APPEND PROPERTY PUBLIC_HEADER GaitTrajectory.hpp)
    target_link_libraries(TrajectoryLib PUBLIC ROBOTICSLAB::Gait)
endif()

install(TARGETS TrajectoryLib
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Exporting dependencies for ASIBOTConfig.cmake quite manually for now...
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS} ${GAIT_LINK_DIRS} CACHE INTERNAL "appended libraries")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES} ${KEYWORD} CACHE INTERNAL "appended libraries")

endif()
