option(ENABLE_KdlVectorConverterLib "Enable/disable compilation of KdlVectorConverterLib" ON)

if(ENABLE_KdlVectorConverterLib)

find_package(orocos_kdl REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})

# Set up our library.
add_library(KdlVectorConverterLib KdlVectorConverter.cpp
                                  KdlVectorConverter.hpp)

target_link_libraries(KdlVectorConverterLib ${orocos_kdl_LIBRARIES})

install(TARGETS KdlVectorConverterLib
        DESTINATION lib)

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES} ${KEYWORD} CACHE INTERNAL "appended libraries")

endif()
