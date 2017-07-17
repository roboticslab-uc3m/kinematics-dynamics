option(ENABLE_KdlVectorConverterLib "Enable/disable KdlVectorConverterLib library" ON)

if(ENABLE_KdlVectorConverterLib)

find_package(orocos_kdl REQUIRED)

# Set up our library.
add_library(KdlVectorConverterLib KdlVectorConverter.cpp
                                  KdlVectorConverter.hpp)

set_target_properties(KdlVectorConverterLib PROPERTIES PUBLIC_HEADER KdlVectorConverter.hpp)

target_link_libraries(KdlVectorConverterLib PUBLIC ${orocos_kdl_LIBRARIES})

target_include_directories(KdlVectorConverterLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS KdlVectorConverterLib
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES} ${KEYWORD} CACHE INTERNAL "appended libraries")

endif()
