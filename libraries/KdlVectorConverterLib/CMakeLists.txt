verbose_dependent_option(KdlVectorConverterLib DEPENDS_WARNING orocos_kdl_FOUND)

if(ENABLE_KdlVectorConverterLib)

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

endif()
