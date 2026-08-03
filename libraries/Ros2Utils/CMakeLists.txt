if(ENABLE_CartesianControlClientROS2 OR ENABLE_CartesianControlServerROS2)
    add_library(Ros2Utils OBJECT Ros2Utils.hpp
                                 Ros2Utils.cpp)

    set_target_properties(Ros2Utils PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

    target_link_libraries(Ros2Utils PUBLIC YARP::YARP_os
                                           rclcpp::rclcpp)

    target_include_directories(Ros2Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    add_library(ROBOTICSLAB::Ros2Utils ALIAS Ros2Utils)
endif()
