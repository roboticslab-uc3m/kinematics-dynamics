cmake_dependent_option(ENABLE_KdlVectorConverterLib "Enable/disable KdlVectorConverterLib library" ON
                       orocos_kdl_FOUND OFF)

unset(ENABLE_KdlVectorConverterLib)

if(NOT DEFINED ENABLE_KdlVectorConverterLib)
    set(ENABLE_KdlVectorConverterLib ON CACHE BOOL "")
endif()

if(NOT orocos_kdl_FOUND)
    if(ENABLE_KdlVectorConverterLib)
        message(WARNING "orocos_kdl package not found, disabling KdlVectorConverterLib")
    endif()
    if(NOT DEFINED ENABLE_KdlVectorConverterLib-STORED)
        set(ENABLE_KdlVectorConverterLib-STORED "${ENABLE_KdlVectorConverterLib}" CACHE INTERNAL "Stored value of ENABLE_KdlVectorConverterLib prior to forcing OFF")
    endif()
    set(ENABLE_KdlVectorConverterLib OFF CACHE BOOL "Enable/disable KdlVectorConverterLib" FORCE)
elseif(DEFINED ENABLE_KdlVectorConverterLib-STORED AND NOT ENABLE_KdlVectorConverterLib)
    set(ENABLE_KdlVectorConverterLib "${ENABLE_KdlVectorConverterLib-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_KdlVectorConverterLib-STORED CACHE)
endif()

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
