if(NOT DEFINED ENABLE_KdlSolver OR ENABLE_KdlSolver)
    if(NOT orocos_kdl_FOUND)
        message(WARNING "orocos_kdl package not found, disabling KdlSolver")
    elseif(NOT SCREW_THEORY_SOLVERS_FOUND)
        message(WARNING "SCREW_THEORY_SOLVERS package not found, disabling KdlSolver")
    endif()
endif()

yarp_prepare_plugin(KdlSolver
                    CATEGORY device
                    TYPE KdlSolver
                    INCLUDE KdlSolver.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND;SCREW_THEORY_SOLVERS_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_KdlSolver)

    yarp_add_plugin(KdlSolver)

    target_sources(KdlSolver PRIVATE KdlSolver.hpp
                                     DeviceDriverImpl.cpp
                                     ICartesianSolverImpl.cpp
                                     ChainFkSolverPos_ST.hpp
                                     ChainFkSolverPos_ST.cpp
                                     ChainIkSolverPos_ST.hpp
                                     ChainIkSolverPos_ST.cpp
                                     ChainIkSolverPos_ID.hpp
                                     ChainIkSolverPos_ID.cpp
                                     KdlSolver_ParamsParser.h
                                     KdlSolver_ParamsParser.cpp
                                     LogComponent.hpp
                                     LogComponent.cpp)

    target_link_libraries(KdlSolver YARP::YARP_os
                                    YARP::YARP_dev
                                    ROBOTICSLAB::ScrewTheoryLib
                                    ROBOTICSLAB::KdlVectorConverterLib
                                    ROBOTICSLAB::KinematicsDynamicsInterfaces
                                    ROBOTICSLAB::KdlSolverUtils)

    yarp_install(TARGETS KdlSolver
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_KdlSolver OFF CACHE BOOL "Enable/disable KdlSolver device" FORCE)

endif()
