if(ENABLE_CartesianControlClientROS2 OR ENABLE_CartesianControlServerROS2)
    add_library(RosUtils OBJECT Spinner.hpp
                                Spinner.cpp)

    set_target_properties(RosUtils PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

    target_link_libraries(RosUtils PUBLIC YARP::YARP_os
                                          rclcpp::rclcpp)

    target_include_directories(RosUtils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    add_library(ROBOTICSLAB::RosUtils ALIAS RosUtils)
endif()
