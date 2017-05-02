yarp_prepare_plugin(AsibotSolver
                    CATEGORY device
                    TYPE roboticslab::AsibotSolver
                    INCLUDE AsibotSolver.hpp)

if(NOT SKIP_AsibotSolver)

find_package(YARP REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

yarp_add_plugin(AsibotSolver AsibotSolver.hpp
                             DeviceDriverImpl.cpp
                             ICartesianSolverImpl.cpp
                             ICartesianImpl.cpp
                             RateThreadImpl.cpp)

target_link_libraries(AsibotSolver ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES} ${YARP_LIBRARIES})

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")

yarp_install(TARGETS AsibotSolver
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES AsibotSolver.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}) 

endif()
