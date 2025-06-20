# https://stackoverflow.com/a/25217937
get_directory_property(_has_parent PARENT_DIRECTORY)

if(_has_parent)
  option(ENABLE_ros2_ws_spacenav_device "Enable/disable spacenav_device ROS 2 package" ON)

  if(NOT ENABLE_ros2_ws_spacenav_device)
    return()
  endif()
endif()

cmake_minimum_required(VERSION 3.8)
project(spacenav_device)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include directories
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
# avoid due to conflict with ROS-vendored orocos-kdl package
# find_package(tf2_geometry_msgs REQUIRED)

if(NOT TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
  find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

add_executable(spacenav_device src/spacenav_device.cpp)

target_link_libraries(spacenav_device ROBOTICSLAB::KinematicsDynamicsInterfaces)

ament_target_dependencies(spacenav_device rclcpp std_msgs sensor_msgs geometry_msgs tf2)

install(TARGETS
  spacenav_device
  DESTINATION lib/${PROJECT_NAME})

ament_package()
