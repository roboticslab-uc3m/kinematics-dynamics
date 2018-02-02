find_package(orocos_kdl ${_target_kdl_version} QUIET)

if(ENABLE_KinematicRepresentationLib AND NOT orocos_kdl_FOUND)
    message(WARNING "orocos_kdl package not found, disabling KinematicRepresentationLib")
    set(ENABLE_KinematicRepresentationLib OFF CACHE BOOL "Enable/disable KinematicRepresentationLib library" FORCE)
    return()
endif()

option(ENABLE_KinematicRepresentationLib "Enable/disable KinematicRepresentationLib library" ON)

if(ENABLE_KinematicRepresentationLib)

add_library(KinematicRepresentationLib KinematicRepresentation.hpp
                                       KinematicRepresentation.cpp)

add_dependencies(KinematicRepresentationLib COLOR_DEBUG)

set_target_properties(KinematicRepresentationLib PROPERTIES PUBLIC_HEADER KinematicRepresentation.hpp)

target_link_libraries(KinematicRepresentationLib PRIVATE ${orocos_kdl_LIBRARIES})

target_include_directories(KinematicRepresentationLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                                      PRIVATE ${orocos_kdl_INCLUDE_DIRS})

install(TARGETS KinematicRepresentationLib
        EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS KinematicRepresentationLib)

endif()
