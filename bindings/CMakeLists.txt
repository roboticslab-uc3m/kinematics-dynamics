# UseSWIG generates now standard target names.
cmake_policy(SET CMP0078 OLD)

# UseSWIG honors SWIG_MODULE_NAME via -module flag.
cmake_policy(SET CMP0086 OLD)

include(CMakeDependentOption)

# disable all warnings for current folder and subfolders
# (unless explicitly added by the user)
get_property(_USER_CMAKE_C_FLAGS CACHE CMAKE_C_FLAGS PROPERTY VALUE)
get_property(_USER_CMAKE_CXX_FLAGS CACHE CMAKE_CXX_FLAGS PROPERTY VALUE)
set(CMAKE_C_FLAGS "-w ${_USER_CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-w ${_USER_CMAKE_CXX_FLAGS}")

# This is necessary for SWIG to parse .i file
#get_property(_include_dirs TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
#foreach(_dir IN LISTS _include_dirs)
#  if("${_dir}" MATCHES "\\\$<BUILD_INTERFACE:(.+)>$")
#    include_directories("${CMAKE_MATCH_1}")
#  elseif("${_dir}" MATCHES "\\\$<INSTALL_INTERFACE:(.+)>$")
    # Nothing to do
#  else()
#    include_directories(${_dir})
#  endif()
#endforeach()

cmake_dependent_option(CREATE_BINDINGS "Compile optional language bindings" ON
                       SWIG_FOUND OFF)

if(CREATE_BINDINGS)
  # Allow passing extra flags to swig (e.g. -Wall)
  set(SWIG_EXTRA_FLAGS "" CACHE STRING "Extra flags passed to swig commands (e.g. -Wall)")
  mark_as_advanced(SWIG_EXTRA_FLAGS)

  set(SWIG_COMMON_FLAGS "${SWIG_EXTRA_FLAGS}")

  set(CREATE_BINDINGS_PYTHON FALSE CACHE BOOL "Create Python interface")

  # Include SWIG use file
  include(${SWIG_USE_FILE})

  set(SWIG_RL_LIBRARIES YARP::YARP_dev
                        ROBOTICSLAB::KinematicsDynamicsInterfaces)

  set(SWIG_BINDINGS_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/roboticslab_kinematics_dynamics.i")

  if(CREATE_BINDINGS_PYTHON)
    add_subdirectory(python)
  endif()
endif()
