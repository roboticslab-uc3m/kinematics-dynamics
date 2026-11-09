if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_hapticFeedback OR ENABLE_hapticFeedback))
    message(WARNING "orocos_kdl package not found, disabling hapticFeedback")
endif()

cmake_dependent_option(ENABLE_hapticFeedback "Enable/disable hapticFeedback program" ON
                       "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND" OFF)

if(ENABLE_hapticFeedback)

    add_executable(hapticFeedback main.cpp
                                  HapticFeedback.hpp
                                  HapticFeedback.cpp
                                  LogComponent.hpp
                                  LogComponent.cpp)

    target_link_libraries(hapticFeedback ${orocos_kdl_LIBRARIES}
                                         YARP::YARP_os
                                         YARP::YARP_init
                                         YARP::YARP_dev
                                         ROBOTICSLAB::KdlVectorConverterLib
                                         ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(hapticFeedback PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS hapticFeedback)

else()

    set(ENABLE_hapticFeedback OFF CACHE BOOL "Enable/disable hapticFeedback program" FORCE)

endif()
