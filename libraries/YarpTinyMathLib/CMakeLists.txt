option(ENABLE_YarpTinyMathLib "Enable/disable YarpTinyMathLib library" ON)

if(ENABLE_YarpTinyMathLib)

find_package(YARP REQUIRED)

add_library(YarpTinyMathLib YarpTinyMath.hpp
                            YarpTinyMath.cpp)

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

endif()
