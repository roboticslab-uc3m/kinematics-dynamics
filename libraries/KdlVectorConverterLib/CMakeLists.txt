option(ENABLE_KdlVectorConverterLib "Enable/disable KdlVectorConverterLib library" ON)

find_package(orocos_kdl ${_target_kdl_version} QUIET)

if(ENABLE_KdlVectorConverterLib)

if(NOT orocos_kdl_FOUND)
    message(WARNING "orocos_kdl package not found, disabling KdlVectorConverterLib")
    set(ENABLE_KdlVectorConverterLib OFF CACHE BOOL "Enable/disable KdlVectorConverterLib library" FORCE)
    return()
endif()

# Set up our library.
add_library(KdlVectorConverterLib KdlVectorConverter.cpp
                                  KdlVectorConverter.hpp)

add_dependencies(KdlVectorConverterLib COLOR_DEBUG)

set_target_properties(KdlVectorConverterLib PROPERTIES PUBLIC_HEADER KdlVectorConverter.hpp)

target_link_libraries(KdlVectorConverterLib PUBLIC ${orocos_kdl_LIBRARIES})

target_include_directories(KdlVectorConverterLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS KdlVectorConverterLib
        EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS KdlVectorConverterLib)

endif()
