if(ENABLE_KdlSolver OR ENABLE_KdlTreeSolver)
    add_library(KdlSolverUtils OBJECT KdlSolverUtils.hpp
                                      KdlSolverUtils.cpp)

    set_target_properties(KdlSolverUtils PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

    target_link_libraries(KdlSolverUtils PUBLIC YARP::YARP_os
                                         ${orocos_kdl_LIBRARIES})

    target_include_directories(KdlSolverUtils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                                                     ${orocos_kdl_INCLUDE_DIRS})

    add_library(ROBOTICSLAB::KdlSolverUtils ALIAS KdlSolverUtils)
endif()
