if(NOT YARP_idl_tools_FOUND AND (NOT DEFINED ENABLE_CartesianIDL OR ENABLE_CartesianIDL))
    message(WARNING "YARP idl_tools component not found, disabling CartesianIDL library")
endif()

cmake_dependent_option(ENABLE_CartesianIDL "Enable/disable CartesianIDL library" ON
                       YARP_idl_tools_FOUND OFF)

if(ENABLE_CartesianIDL)

    set(ALLOW_IDL_GENERATION ON CACHE BOOL "Detect changes and rebuild IDL files")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS cartesian.thrift)

    yarp_idl_to_dir(INPUT_FILES cartesian.thrift
                    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
                    SOURCES_VAR idl_sources
                    HEADERS_VAR idl_headers
                    INCLUDE_DIRS_VAR idl_include_dirs
                    PLACEMENT MERGED
                    THRIFT_NO_NAMESPACE_PREFIX)

    add_library(CartesianIDL SHARED ${idl_sources} ${idl_headers})

    set_target_properties(CartesianIDL PROPERTIES PUBLIC_HEADER "${idl_headers}"
                                                  WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    target_link_libraries(CartesianIDL PUBLIC YARP::YARP_dev
                                              ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(CartesianIDL PUBLIC $<BUILD_INTERFACE:${idl_include_dirs}>
                                                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

    install(TARGETS CartesianIDL
            EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS)

    set_property(GLOBAL APPEND PROPERTY _exported_targets CartesianIDL)

    add_library(ROBOTICSLAB::CartesianIDL ALIAS CartesianIDL)

else()

    set(ENABLE_CartesianIDL OFF CACHE BOOL "Enable/disable CartesianIDL library" FORCE)

endif()
