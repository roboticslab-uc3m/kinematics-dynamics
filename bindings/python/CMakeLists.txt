# this is set in the parent directory, but for some reason it does not get inherited
set_source_files_properties(${SWIG_BINDINGS_SOURCE_FILE} PROPERTIES CPLUSPLUS ON)

set(CMAKE_SWIG_FLAGS "-module;roboticslab_kinematics_dynamics;-threads;${SWIG_COMMON_FLAGS}")

# https://github.com/pybind/pybind11/pull/2689
if(${CMAKE_VERSION} VERSION_LESS "3.18.0")
  set(RL_Python_Development_Component "Development")
else()
  set(RL_Python_Development_Component "Development.Module")
endif()

find_package(Python3 COMPONENTS Interpreter ${RL_Python_Development_Component} REQUIRED)

set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
set(SWIG_OUTFILE_DIR "${CMAKE_CURRENT_BINARY_DIR}")

swig_add_library(roboticslab_kinematics_dynamics_python
                 LANGUAGE python
                 SOURCES ${SWIG_BINDINGS_SOURCE_FILE})
swig_link_libraries(roboticslab_kinematics_dynamics_python Python3::Module ${SWIG_RL_LIBRARIES})

set_target_properties(${SWIG_MODULE_roboticslab_kinematics_dynamics_python_REAL_NAME} PROPERTIES OUTPUT_NAME "_roboticslab_kinematics_dynamics"
                                                                                                 LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python")

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
              ${CMAKE_CURRENT_SOURCE_DIR}/kinematics_dynamics.py # deprecation
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
