# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
link_directories(${YARP_LIBRARY_DIRS})
link_libraries(${YARP_LIBRARIES})

find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
include_directories(${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS})
link_directories(${ROBOTICSLAB_KINEMATICS_DYNAMICS_LINK_DIRS})
link_libraries(${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES})

# Work-around for missing paths to OpenCV libraries
find_package(OpenCV)
if(OpenCV_FOUND)
  link_directories(${OpenCV_LINK_DIRECTORIES} ${OPENCV_LINK_DIRECTORIES})
endif()

include_directories(${CMAKE_SOURCE_DIR}/../src/libraries/ColorClientLib)
include_directories(/usr/local/include)

# for yarp.i
include_directories($ENV{YARP_ROOT}/bindings)

set(CREATE_PYTHON FALSE CACHE BOOL "Create Python interface")
set(CREATE_RUBY FALSE CACHE BOOL "Create Ruby interface")
set(CREATE_JAVA FALSE CACHE BOOL "Create Java interface")
SET(CREATE_CSHARP FALSE CACHE BOOL "Create CSharp interface")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
message(STATUS "${SWIG_USE_FILE}")
set(MIN_SWIG_VERSION "1.3.29")

if("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")
  message(FATAL_ERROR "Please update your SWIG. Current version: ${SWIG_VERSION}. The minimum version needed: ${MIN_SWIG_VERSION}")
endif()

set_source_files_properties(kinematics_dynamics.i PROPERTIES CPLUSPLUS ON)

if(CREATE_PYTHON)
  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}")
  set(CMAKE_SWIG_FLAGS "-Wall;-module;kinematics_dynamics;-threads")
  set(CREATE_PYTHON_VERSION "" CACHE STRING "Control python version used, if you wish." )
  find_package(PythonLibs ${CREATE_PYTHON_VERSION})
  include_directories(${PYTHON_INCLUDE_PATH})
  link_libraries(${PYTHON_LIBRARY})

  ## in Windows it seems necessary to declare explictly the link_directory
  if(WIN32)
    get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
    link_directories(${PYTHON_DIR})
  endif()

  swig_add_module(kinematics_dynamics python kinematics_dynamics.i)

  # uncomment if we want a debug version of the library
  #add_definitions(-DPy_DEBUG)

  # installation path is determined reliably on most platforms using distutils
  find_package(PythonInterp ${CREATE_PYTHON_VERSION})

  execute_process(COMMAND ${PYTHON_EXECUTABLE}
    -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix='${CMAKE_INSTALL_PREFIX}'))" 
    OUTPUT_VARIABLE PYTHON_INSTDIR OUTPUT_STRIP_TRAILING_WHITESPACE )
  install(FILES ${CMAKE_BINARY_DIR}/kinematics_dynamics.py ${CMAKE_BINARY_DIR}/_kinematics_dynamics.so 
    DESTINATION ${PYTHON_INSTDIR} )

  if(WIN32)
    # Check if we have a target called _kinematics_dynamics as SWIG_ADD_MODULE
    # will currently give. If not, we're probably in the distant future, and
    # we'd best not meddle.
    get_target_property(kinematics_dynamics_lib_location _kinematics_dynamics LOCATION)
    if(kinematics_dynamics_lib_location)
      set_target_properties(_kinematics_dynamics PROPERTIES SUFFIX ".pyd")
    endif()
  endif()
endif()

if(CREATE_RUBY)
  find_package(Ruby REQUIRED)
  set(CMAKE_SWIG_FLAGS "-Wall;-module;kinematics_dynamics")
  include_directories(${RUBY_INCLUDE_PATH})
  swig_add_module(kinematics_dynamics ruby kinematics_dynamics.i)
  swig_link_libraries(kinematics_dynamics ${RUBY_LIBRARY})
  set_target_properties(kinematics_dynamics PROPERTIES PREFIX "")
endif()

if(CREATE_JAVA)
  option(PREPARE_CLASS_FILES "Prepare class files" OFF)
  set(ROBOTICSLAB_KINEMATICS_DYNAMICS_HELPERS LoadAsibot.java)
  make_directory("${CMAKE_CURRENT_BINARY_DIR}/kinematics_dynamics")
  set(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/kinematics_dynamics")
  set(TARGET_DIR src)

  if(NOT BINDINGS_SRC_DIR)
    set(BINDINGS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endif()

  if(NOT PREPARE_CLASS_FILES)
    set(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/generated_src")
    set(TARGET_DIR generated_src)
  endif()

  make_directory("${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}")
  foreach(src ${ROBOTICSLAB_KINEMATICS_DYNAMICS_HELPERS})
    set(target ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}/${src})
    configure_file(${BINDINGS_SRC_DIR}/${src}
      ${target} IMMEDIATE)
    message(STATUS "copied ${target}")
  endforeach()

  set(CMAKE_SWIG_FLAGS "-package;kinematics_dynamics;-Wall;-module;kinematics_dynamics")
  find_package(JNI REQUIRED)
  include_directories(${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2} )
  set(target_name jkinematics_dynamics)
  swig_add_module(${target_name} java kinematics_dynamics.i)

  if(PREPARE_CLASS_FILES)
    add_custom_command(TARGET ${target_name}
      POST_BUILD
      COMMAND javac -source 1.3 -target 1.3 "src/*.java" "kinematics_dynamics/*.java"
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  # issue on MINGW where C++ name-mangling prevents java finding methods
  if(MINGW)
    message(STATUS "untested MINGW patch - see CMakeLists.txt")
    set_target_properties(${target_name} PROPERTIES
      LINK_FLAGS "--add-stdcall-alias")
  endif()

  ## the following is based on:
  ## http://www.cmake.org/pipermail/cmake/2007-September/016343.html
  if(APPLE)
    set_target_properties(${target_name} PROPERTIES SUFFIX ".jnilib")
  endif()
endif()

if(CREATE_CSHARP)
   set(target_name kinematics_dynamics)
   set(CMAKE_SWIG_FLAGS "-Wall;-module;kinematics_dynamics")
   swig_add_module(kinematics_dynamics csharp kinematics_dynamics.i)
endif()
