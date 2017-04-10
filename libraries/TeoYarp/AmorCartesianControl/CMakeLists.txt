# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(AmorCartesianControl
                    CATEGORY device
                    TYPE roboticslab::AmorCartesianControl
                    INCLUDE AmorCartesianControl.hpp
                    WRAPPER CartesianControlServer)

if(NOT SKIP_AmorCartesianControl)
  find_package(AMOR_API REQUIRED)

  include_directories(${YARP_INCLUDE_DIRS}
                      ${AMOR_API_INCLUDE_DIRS}
                      ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS}
                      ${COLOR_DEBUG_INCLUDE_DIRS}
                      ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(AmorCartesianControl AmorCartesianControl.hpp
                                       DeviceDriverImpl.cpp
                                       ICartesianControlImpl.cpp)

  target_link_libraries(AmorCartesianControl ${YARP_LIBRARIES} ${AMOR_API_LIBRARIES})

  install(TARGETS AmorCartesianControl DESTINATION lib)

  yarp_install(TARGETS AmorCartesianControl
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES AmorCartesianControl.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  # Exporting dependencies for TEOConfig.cmake quite manually for now... 
  set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
endif()
