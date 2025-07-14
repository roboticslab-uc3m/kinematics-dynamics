# this is set in the parent directory, but for some reason it does not get inherited
set_source_files_properties(${SWIG_BINDINGS_SOURCE_FILE} PROPERTIES CPLUSPLUS ON)

set(CMAKE_SWIG_FLAGS "-module;roboticslab_kinematics_dynamics;-threads;${SWIG_COMMON_FLAGS}")

# not using Development.Module here because it conflicts with ROS-coupled CMake (works on 3.28, but not on 3.20)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
set(SWIG_OUTFILE_DIR "${CMAKE_CURRENT_BINARY_DIR}")

swig_add_library(roboticslab_kinematics_dynamics_python
                 LANGUAGE python
                 SOURCES ${SWIG_BINDINGS_SOURCE_FILE})
swig_link_libraries(roboticslab_kinematics_dynamics_python Python3::Module ${SWIG_RL_LIBRARIES})

set_target_properties(${SWIG_MODULE_roboticslab_kinematics_dynamics_python_REAL_NAME} PROPERTIES OUTPUT_NAME "_roboticslab_kinematics_dynamics"
                                                                                                 LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python"
                                                                                                 # treat Python3_INCLUDE_DIRS as non-system so that it can be overriden
                                                                                                 NO_SYSTEM_FROM_IMPORTED TRUE)

# INCLUDE_DIRECTORIES might have gotten polluted by 3rd-party deps, make sure
# the Python3 header path has more predecence by overriding the previous value
# (set by `swig_link_libraries`) and prepending it to the search path
target_include_directories(${SWIG_MODULE_roboticslab_kinematics_dynamics_python_REAL_NAME} BEFORE PRIVATE ${Python3_INCLUDE_DIRS})

# installation path is determined reliably on most platforms using distutils
execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(plat_specific=True,standard_lib=False,prefix=''))"
                OUTPUT_VARIABLE Python_INSTDIR
                OUTPUT_STRIP_TRAILING_WHITESPACE )

set(_CMAKE_INSTALL_PYTHONDIR "${Python_INSTDIR}")
set(CMAKE_INSTALL_PYTHONDIR ${_CMAKE_INSTALL_PYTHONDIR} CACHE PATH "python bindings (${_CMAKE_INSTALL_PYTHONDIR})")
mark_as_advanced(CMAKE_INSTALL_PYTHONDIR)
if(NOT IS_ABSOLUTE ${CMAKE_INSTALL_PYTHONDIR})
  set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_PYTHONDIR}")
else()
  set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PYTHONDIR}")
endif()

install(FILES ${CMAKE_BINARY_DIR}/lib/python/roboticslab_kinematics_dynamics.py
        DESTINATION ${CMAKE_INSTALL_PYTHONDIR})

# Update RPATH
if(NOT CMAKE_SKIP_RPATH AND NOT CMAKE_SKIP_INSTALL_RPATH)
  file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_FULL_PYTHONDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}")
  get_target_property(_current_rpath ${SWIG_MODULE_roboticslab_kinematics_dynamics_python_REAL_NAME} INSTALL_RPATH)
  if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND _current_rpath "@loader_path/${_rel_path}")
  else()
    list(APPEND _current_rpath "\$ORIGIN/${_rel_path}")
  endif()
  set_target_properties(${SWIG_MODULE_roboticslab_kinematics_dynamics_python_REAL_NAME} PROPERTIES INSTALL_RPATH "${_current_rpath}")
endif()

install(TARGETS ${SWIG_MODULE_roboticslab_kinematics_dynamics_python_REAL_NAME}
        DESTINATION ${CMAKE_INSTALL_PYTHONDIR})
