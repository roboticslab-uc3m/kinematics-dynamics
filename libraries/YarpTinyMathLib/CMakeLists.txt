if(NOT YARP_math_FOUND AND (NOT DEFINED ENABLE_YarpTinyMathLib OR ENABLE_YarpTinyMathLib))
    message(WARNING "YARP_math lib not found, disabling YarpTinyMathLib")
endif()

cmake_dependent_option(ENABLE_YarpTinyMathLib "Enable/disable YarpTinyMathLib library" ON
                       YARP_math_FOUND OFF)

if(ENABLE_YarpTinyMathLib)

    add_library(YarpTinyMathLib SHARED YarpTinyMath.hpp
                                       YarpTinyMath.cpp)

    set_target_properties(YarpTinyMathLib PROPERTIES PUBLIC_HEADER YarpTinyMath.hpp)

    target_link_libraries(YarpTinyMathLib PUBLIC YARP::YARP_sig
                                          PRIVATE YARP::YARP_math)

    target_include_directories(YarpTinyMathLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS YarpTinyMathLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS)

    set_property(GLOBAL APPEND PROPERTY _exported_dependencies YARP_sig)

    add_library(ROBOTICSLAB::YarpTinyMathLib ALIAS YarpTinyMathLib)

else()

    set(ENABLE_YarpTinyMathLib OFF CACHE BOOL "Enable/disable YarpTinyMathLib library" FORCE)

endif()
