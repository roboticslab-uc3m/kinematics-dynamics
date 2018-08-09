cmake_dependent_option(ENABLE_YarpTinyMathLib "Enable/disable YarpTinyMathLib library" ON
                       YARP_HAS_MATH_LIB OFF)

unset(ENABLE_YarpTinyMathLib)

if(NOT DEFINED ENABLE_YarpTinyMathLib)
    set(ENABLE_YarpTinyMathLib ON CACHE BOOL "")
endif()

if(NOT YARP_HAS_MATH_LIB)
    if(ENABLE_YarpTinyMathLib)
        message(WARNING "YARP::YARP_math package not found, disabling YarpTinyMathLib")
    endif()
    if(NOT DEFINED ENABLE_YarpTinyMathLib-STORED)
        set(ENABLE_YarpTinyMathLib-STORED "${ENABLE_YarpTinyMathLib}" CACHE INTERNAL "Stored value of ENABLE_YarpTinyMathLib prior to forcing OFF")
    endif()
    set(ENABLE_YarpTinyMathLib OFF CACHE BOOL "Enable/disable YarpTinyMathLib" FORCE)
elseif(DEFINED ENABLE_YarpTinyMathLib-STORED AND NOT ENABLE_YarpTinyMathLib)
    set(ENABLE_YarpTinyMathLib "${ENABLE_YarpTinyMathLib-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_YarpTinyMathLib-STORED CACHE)
endif()

if(ENABLE_YarpTinyMathLib)

    add_library(YarpTinyMathLib SHARED YarpTinyMath.hpp
                                       YarpTinyMath.cpp)

    set_target_properties(YarpTinyMathLib PROPERTIES PUBLIC_HEADER YarpTinyMath.hpp)

    target_link_libraries(YarpTinyMathLib YARP::YARP_sig
                                          YARP::YARP_math)

    target_include_directories(YarpTinyMathLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS YarpTinyMathLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

endif()
