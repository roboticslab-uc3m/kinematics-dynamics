cmake_dependent_option(USE_LOCAL_ROS2_PACKAGES "Build ROS 2 packages locally" OFF
                       rclcpp_FOUND OFF)

cmake_dependent_option(ENABLE_ros2_rl_cartesian_control_msgs "Enable/disable rl_cartesian_control_msgs ROS 2 package" ON
                       USE_LOCAL_ROS2_PACKAGES OFF)

cmake_dependent_option(ENABLE_ros2_spacenav_device "Enable/disable spacenav_device ROS 2 package" ON
                       USE_LOCAL_ROS2_PACKAGES OFF)

if(ENABLE_ros2_rl_cartesian_control_msgs)
    add_subdirectory(workspace/src/rl_cartesian_control_msgs)
endif()

if(ENABLE_ros2_spacenav_device)
    add_subdirectory(workspace/src/spacenav_device)
endif()
