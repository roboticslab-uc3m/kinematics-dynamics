# Created by Juan G Victores 2013 @ UC3M
# Thanks to Paul Fitzpatrick for all the YARP and iCub SWIG stuff for inspiration

cmake_minimum_required(VERSION 2.6)

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
link_directories(${YARP_LIBRARY_DIRS})
link_libraries(${YARP_LIBRARIES})

find_package(TEO REQUIRED)
include_directories(${TEO_INCLUDE_DIRS})
link_directories(${TEO_LINK_DIRS})
link_libraries(${TEO_LIBRARIES})

# Work-around for missing paths to OpenCV libraries
find_package(OpenCV)
if (OpenCV_FOUND)
  link_directories(${OpenCV_LINK_DIRECTORIES} ${OPENCV_LINK_DIRECTORIES})
endif (OpenCV_FOUND)

#MESSAGE(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../src/libraries/ColorClientLib)

# for yarp.i
include_directories($ENV{YARP_ROOT}/bindings)

set(CREATE_PYTHON FALSE CACHE BOOL "Create Python interface")
set(CREATE_RUBY FALSE CACHE BOOL "Create Ruby interface")
set(CREATE_JAVA FALSE CACHE BOOL "Create Java interface")
SET(CREATE_CSHARP FALSE CACHE BOOL "Create CSharp interface")

FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})
MESSAGE(STATUS "${SWIG_USE_FILE}")
SET (MIN_SWIG_VERSION "1.3.29")
IF ("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")
  MESSAGE( FATAL_ERROR "Please update your SWIG. Current version: ${SWIG_VERSION}. The minimum version needed: ${MIN_SWIG_VERSION}")
ENDIF ("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")

SET_SOURCE_FILES_PROPERTIES(teo.i PROPERTIES CPLUSPLUS ON)

if (CREATE_PYTHON)
  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}")
  SET(CMAKE_SWIG_FLAGS "-Wall;-module;teo;-threads")
  SET(CREATE_PYTHON_VERSION "" CACHE STRING "Control python version used, if you wish." )
  FIND_PACKAGE(PythonLibs ${CREATE_PYTHON_VERSION})
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
  LINK_LIBRARIES(${PYTHON_LIBRARY})
  
  ## in Windows it seems necessary to declare explictly the link_directory
  if (WIN32)
    get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
    link_directories(${PYTHON_DIR})
  endif()
  
  SWIG_ADD_MODULE(teo python teo.i)
  # uncomment if we want a debug version of the library
  #ADD_DEFINITIONS(-DPy_DEBUG)

  # installation path is determined reliably on most platforms using distutils
  FIND_PACKAGE(PythonInterp ${CREATE_PYTHON_VERSION})

  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} 
    -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix='${CMAKE_INSTALL_PREFIX}'))" 
    OUTPUT_VARIABLE PYTHON_INSTDIR OUTPUT_STRIP_TRAILING_WHITESPACE )
  INSTALL(FILES ${CMAKE_BINARY_DIR}/teo.py ${CMAKE_BINARY_DIR}/_teo.so 
    DESTINATION ${PYTHON_INSTDIR} )

  IF (WIN32)
    # Check if we have a target called _teo as SWIG_ADD_MODULE
    # will currently give. If not, we're probably in the distant
    # future, and we'd best not meddle.
    GET_TARGET_PROPERTY(teo_lib_location _teo LOCATION)
    IF (teo_lib_location)
      SET_TARGET_PROPERTIES(_teo PROPERTIES SUFFIX ".pyd")
    ENDIF ()    
  ENDIF (WIN32)
endif (CREATE_PYTHON)

if (CREATE_RUBY)
  find_package(Ruby REQUIRED)
  set(CMAKE_SWIG_FLAGS "-Wall;-module;teo")
  include_directories(${RUBY_INCLUDE_PATH})
  swig_add_module(teo ruby teo.i)
  swig_link_libraries(teo ${RUBY_LIBRARY})
  set_target_properties(teo PROPERTIES PREFIX "")
endif (CREATE_RUBY)

if (CREATE_JAVA)
  OPTION(PREPARE_CLASS_FILES "Prepare class files" OFF)
  SET(TEO_HELPERS LoadAsibot.java)
  MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/teo")
  SET(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/teo")
  SET(TARGET_DIR src)
  IF (NOT BINDINGS_SRC_DIR)
    SET(BINDINGS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ENDIF ()
  IF (NOT PREPARE_CLASS_FILES)
    SET(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/generated_src")
    SET(TARGET_DIR generated_src)
  ENDIF ()
  MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}")
  FOREACH(src ${TEO_HELPERS})
    SET(target ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}/${src})
    CONFIGURE_FILE(${BINDINGS_SRC_DIR}/${src}
      ${target} IMMEDIATE)
    MESSAGE(STATUS "copied ${target}")
  ENDFOREACH(src)
  SET(CMAKE_SWIG_FLAGS "-package;teo;-Wall;-module;teo")
  FIND_PACKAGE(JNI REQUIRED)
  INCLUDE_DIRECTORIES(${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2} )
  SET(target_name jteo)
  SWIG_ADD_MODULE(${target_name} java teo.i)
  IF (PREPARE_CLASS_FILES)
    ADD_CUSTOM_COMMAND(TARGET ${target_name}
      POST_BUILD
      COMMAND javac -source 1.3 -target 1.3 "src/*.java" "teo/*.java"
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ENDIF ()

  # issue on MINGW where C++ name-mangling prevents java finding methods
  IF (MINGW)
    MESSAGE(STATUS "untested MINGW patch - see CMakeLists.txt")
    SET_TARGET_PROPERTIES(${target_name} PROPERTIES 
      LINK_FLAGS "--add-stdcall-alias")
  ENDIF (MINGW)
  ## the following is based on:
  ## http://www.cmake.org/pipermail/cmake/2007-September/016343.html
  IF(APPLE)
    SET_TARGET_PROPERTIES(${target_name} PROPERTIES SUFFIX ".jnilib")
  ENDIF(APPLE)
endif (CREATE_JAVA)

IF (CREATE_CSHARP)
   SET(target_name teo)
   SET(CMAKE_SWIG_FLAGS "-Wall;-module;teo")
   SWIG_ADD_MODULE(teo csharp teo.i)
ENDIF (CREATE_CSHARP)

