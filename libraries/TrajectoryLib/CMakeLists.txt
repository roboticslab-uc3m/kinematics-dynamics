option(ENABLE_TrajectoryLib "Choose if you want to compile TrajectoryLib" ON)

if(ENABLE_TrajectoryLib)

find_package(orocos_kdl REQUIRED)
find_package(GAIT REQUIRED)

add_definitions(${GAIT_DEFINES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${GAIT_INCLUDE_DIRS}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

# Set up our library.
add_library(TrajectoryLib Trajectory.hpp
                          LineTrajectory.cpp
                          LineTrajectory.hpp
                          GaitTrajectory.cpp
                          GaitTrajectory.hpp)

target_link_libraries(TrajectoryLib ${orocos_kdl_LIBRARIES}
                                    ${GAIT_LIBRARIES}
                                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES})

install(TARGETS TrajectoryLib
        DESTINATION lib)

# Exporting dependencies for ASIBOTConfig.cmake quite manually for now...
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS} ${GAIT_LINK_DIRS} CACHE INTERNAL "appended libraries")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES} ${KEYWORD} CACHE INTERNAL "appended libraries")

endif()
