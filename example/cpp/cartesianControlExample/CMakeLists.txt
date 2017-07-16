cmake_minimum_required(VERSION 2.6.0)

# Start a project.
project(cartesianControlExample)

find_package(YARP REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

# Set up our main executable.
add_executable(cartesianControlExample CartesianControlExample.cpp)

target_link_libraries(cartesianControlExample ${YARP_LIBRARIES}
                                              ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES})

install(TARGETS cartesianControlExample
        DESTINATION bin)
