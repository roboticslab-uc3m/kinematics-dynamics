yarp_prepare_plugin(AsibotSolver
                    CATEGORY device
                    TYPE roboticslab::AsibotSolver
                    INCLUDE AsibotSolver.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_YarpTinyMathLib)

if(NOT SKIP_AsibotSolver)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

yarp_add_plugin(AsibotSolver AsibotSolver.hpp
                             DeviceDriverImpl.cpp
                             ICartesianSolverImpl.cpp
                             ICartesianImpl.cpp
                             RateThreadImpl.cpp)

add_dependencies(AsibotSolver COLOR_DEBUG)

target_link_libraries(AsibotSolver YARP::YARP_OS
                                   YARP::YARP_dev
                                   YarpTinyMathLib)

yarp_install(TARGETS AsibotSolver
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES AsibotSolver.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

endif()
