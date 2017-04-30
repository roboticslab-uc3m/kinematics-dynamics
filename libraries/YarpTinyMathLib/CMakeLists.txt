if(ENABLE_YarpTinyMathLib)

set(KEYWORD "YarpTinyMathLib")

find_package(YARP REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})

add_library(${KEYWORD} YarpTinyMath.hpp YarpTinyMath.cpp)
target_link_libraries(${KEYWORD} ${YARP_LIBRARIES})
install(TARGETS ${KEYWORD} DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES} ${KEYWORD} CACHE INTERNAL "appended libraries")

endif()
