# Install script for directory: /home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/src/kdl_parser-noetic-devel/kdl_parser_py

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/build/kdl_parser-noetic-devel/kdl_parser_py/catkin_generated/installspace/kdl_parser_py.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kdl_parser_py/cmake" TYPE FILE FILES
    "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/build/kdl_parser-noetic-devel/kdl_parser_py/catkin_generated/installspace/kdl_parser_pyConfig.cmake"
    "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/build/kdl_parser-noetic-devel/kdl_parser_py/catkin_generated/installspace/kdl_parser_pyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kdl_parser_py" TYPE FILE FILES "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/src/kdl_parser-noetic-devel/kdl_parser_py/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/build/kdl_parser-noetic-devel/kdl_parser_py/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/kdl_parser_py" TYPE PROGRAM FILES "/home/jgl20/work/workssrc/4myarms/src/remote_control-x86-can-v2/follow2/build/kdl_parser-noetic-devel/kdl_parser_py/catkin_generated/installspace/urdf.py")
endif()

