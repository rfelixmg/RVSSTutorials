# Install script for directory: /home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/src/pybot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/build/pybot/catkin_generated/installspace/pybot.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pybot/cmake" TYPE FILE FILES
    "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/build/pybot/catkin_generated/installspace/pybotConfig.cmake"
    "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/build/pybot/catkin_generated/installspace/pybotConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pybot" TYPE FILE FILES "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/src/pybot/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pybot" TYPE PROGRAM FILES
    "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/src/pybot/scripts/pybot_camera.py"
    "/home/chuong/Documents/RVSS2017-presentations/python_ROS_tutorials/workspace_ros/src/pybot/scripts/pybot_vision.py"
    )
endif()

