if((NOT Python3_FOUND OR NOT pybind11_FOUND) AND (NOT DEFINED CREATE_BINDINGS_PYTHON OR CREATE_BINDINGS_PYTHON))
    message(WARNING "Python3/pybind11 package not found, disabling bindings")
endif()

cmake_dependent_option(CREATE_BINDINGS_PYTHON "Enable/disable Python bindings" ON
                       "Python3_FOUND;pybind11_FOUND" OFF)

if(CREATE_BINDINGS_PYTHON)
    set(PYBIND11_PYTHON_VERSION 3 CACHE STRING "Python version used by PyBind11")

    if(Python3_VERSION VERSION_GREATER_EQUAL 3.12)
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import os;import sysconfig;relative_site_packages = sysconfig.get_path('purelib').replace(sysconfig.get_path('data'), '').lstrip(os.path.sep);print(relative_site_packages)"
                        OUTPUT_VARIABLE PYTHON_PACKAGES_INSTALL_DIR
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
                        OUTPUT_VARIABLE PYTHON_PACKAGES_INSTALL_DIR
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()

    set(PY_MODULE_NAME roboticslab_kinematics_dynamics)

    pybind11_add_module(${PY_MODULE_NAME} module.cpp
                                          ICartesianControl.cpp
                                          ICartesianSolver.cpp)

    target_link_libraries(${PY_MODULE_NAME} PRIVATE ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS ${PY_MODULE_NAME}
            DESTINATION "${PYTHON_PACKAGES_INSTALL_DIR}")
else()
    set(CREATE_BINDINGS_PYTHON OFF CACHE BOOL "Enable/disable Python bindings" FORCE)
endif()
