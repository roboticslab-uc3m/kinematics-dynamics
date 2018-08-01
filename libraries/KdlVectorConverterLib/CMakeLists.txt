if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_KdlVectorConverterLib OR ENABLE_KdlVectorConverterLib))
    message(WARNING "orocos_kdl package not found, disabling KdlVectorConverterLib")
endif()

cmake_dependent_option(ENABLE_KdlVectorConverterLib "Enable/disable KdlVectorConverterLib library" ON
                       orocos_kdl_FOUND OFF)

if(ENABLE_KdlVectorConverterLib)

    # Set up our library.
    add_library(KdlVectorConverterLib SHARED KdlVectorConverter.cpp
                                             KdlVectorConverter.hpp)

    set_target_properties(KdlVectorConverterLib PROPERTIES PUBLIC_HEADER KdlVectorConverter.hpp)

    target_link_libraries(KdlVectorConverterLib PUBLIC ${orocos_kdl_LIBRARIES}
                                                PRIVATE ROBOTICSLAB::ColorDebug)

    target_include_directories(KdlVectorConverterLib PUBLIC ${orocos_kdl_INCLUDE_DIRS}
                                                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS KdlVectorConverterLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

else()

    set(ENABLE_KdlVectorConverterLib OFF CACHE BOOL "Enable/disable KdlVectorConverterLib library" FORCE)

endif()
