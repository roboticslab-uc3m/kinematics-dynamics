if(NOT YARP_HAS_MATH_LIB AND (NOT DEFINED ENABLE_YarpTinyMathLib OR ENABLE_YarpTinyMathLib))
    message(WARNING "YARP math lib not found, disabling YarpTinyMathLib")
endif()

cmake_dependent_option(ENABLE_YarpTinyMathLib "Enable/disable YarpTinyMathLib library" ON
                       YARP_HAS_MATH_LIB OFF)

if(ENABLE_YarpTinyMathLib)

    add_library(YarpTinyMathLib YarpTinyMath.hpp
                                YarpTinyMath.cpp)

    set_target_properties(YarpTinyMathLib PROPERTIES PUBLIC_HEADER YarpTinyMath.hpp)

    target_link_libraries(YarpTinyMathLib YARP::YARP_sig
                                          YARP::YARP_math)

    target_include_directories(YarpTinyMathLib PUBLIC ${YARP_INCLUDE_DIRS}
                                                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS YarpTinyMathLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set_property(GLOBAL APPEND PROPERTY ROBOTICSLAB_KINEMATICS_DYNAMICS_TARGETS YarpTinyMathLib)

else()

    set(ENABLE_YarpTinyMathLib OFF CACHE BOOL "Enable/disable YarpTinyMathLib library" FORCE)

endif()
