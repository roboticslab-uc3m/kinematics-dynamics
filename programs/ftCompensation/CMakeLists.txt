if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_ftCompensation OR ENABLE_ftCompensation))
    message(WARNING "orocos_kdl package not found, disabling ftCompensation")
endif()

cmake_dependent_option(ENABLE_ftCompensation "Enable/disable ftCompensation program" ON
                       "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND" OFF)

if(ENABLE_ftCompensation)

    add_executable(ftCompensation main.cpp
                                  FtCompensation.hpp
                                  FtCompensation.cpp)

    target_link_libraries(ftCompensation ${orocos_kdl_LIBRARIES}
                                         YARP::YARP_os
                                         YARP::YARP_init
                                         YARP::YARP_dev
                                         ROBOTICSLAB::KdlVectorConverterLib
                                         ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(ftCompensation PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS ftCompensation)

else()

    set(ENABLE_ftCompensation OFF CACHE BOOL "Enable/disable ftCompensation program" FORCE)

endif()
