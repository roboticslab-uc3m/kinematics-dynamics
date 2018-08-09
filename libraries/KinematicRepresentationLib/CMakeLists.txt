cmake_dependent_option(ENABLE_KinematicRepresentationLib "Enable/disable KinematicRepresentationLib library" ON
                       orocos_kdl_FOUND OFF)

unset(ENABLE_KinematicRepresentationLib)

if(NOT DEFINED ENABLE_KinematicRepresentationLib)
    set(ENABLE_KinematicRepresentationLib ON CACHE BOOL "")
endif()

if(NOT orocos_kdl_FOUND)
    if(ENABLE_KinematicRepresentationLib)
        message(WARNING "orocos_kdl package not found, disabling KinematicRepresentationLib")
    endif()
    if(NOT DEFINED ENABLE_KinematicRepresentationLib-STORED)
        set(ENABLE_KinematicRepresentationLib-STORED "${ENABLE_KinematicRepresentationLib}" CACHE INTERNAL "Stored value of ENABLE_KinematicRepresentationLib prior to forcing OFF")
    endif()
    set(ENABLE_KinematicRepresentationLib OFF CACHE BOOL "Enable/disable KinematicRepresentationLib" FORCE)
elseif(DEFINED ENABLE_KinematicRepresentationLib-STORED AND NOT ENABLE_KinematicRepresentationLib)
    set(ENABLE_KinematicRepresentationLib "${ENABLE_KinematicRepresentationLib-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_KinematicRepresentationLib-STORED CACHE)
endif()

if(ENABLE_KinematicRepresentationLib)

    add_library(KinematicRepresentationLib SHARED KinematicRepresentation.hpp
                                                  KinematicRepresentation.cpp)

    set_target_properties(KinematicRepresentationLib PROPERTIES PUBLIC_HEADER KinematicRepresentation.hpp)

    target_link_libraries(KinematicRepresentationLib PRIVATE ${orocos_kdl_LIBRARIES}
                                                             ROBOTICSLAB::ColorDebug)

    target_include_directories(KinematicRepresentationLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                                 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                                          PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS KinematicRepresentationLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

endif()
