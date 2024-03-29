if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_KinematicRepresentationLib OR ENABLE_KinematicRepresentationLib))
    message(WARNING "orocos_kdl package not found, disabling KinematicRepresentationLib")
endif()

cmake_dependent_option(ENABLE_KinematicRepresentationLib "Enable/disable KinematicRepresentationLib library" ON
                       orocos_kdl_FOUND OFF)

if(ENABLE_KinematicRepresentationLib)

    add_library(KinematicRepresentationLib SHARED KinematicRepresentation.hpp
                                                  KinematicRepresentation.cpp)

    set_target_properties(KinematicRepresentationLib PROPERTIES PUBLIC_HEADER KinematicRepresentation.hpp)

    target_link_libraries(KinematicRepresentationLib PRIVATE ${orocos_kdl_LIBRARIES}
                                                             YARP::YARP_os)

    target_include_directories(KinematicRepresentationLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                                 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                                          PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    target_compile_features(KinematicRepresentationLib PUBLIC cxx_std_17) # nested namespace definition

    install(TARGETS KinematicRepresentationLib
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS)

    add_library(ROBOTICSLAB::KinematicRepresentationLib ALIAS KinematicRepresentationLib)

else()

    set(ENABLE_KinematicRepresentationLib OFF CACHE BOOL "Enable/disable KinematicRepresentationLib library" FORCE)

endif()
