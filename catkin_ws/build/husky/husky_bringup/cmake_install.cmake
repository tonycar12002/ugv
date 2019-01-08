# Install script for directory: /home/tony/ugv/catkin_ws/src/husky/husky_bringup

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tony/ugv/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tony/ugv/catkin_ws/build/husky/husky_bringup/catkin_generated/installspace/husky_bringup.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husky_bringup/cmake" TYPE FILE FILES
    "/home/tony/ugv/catkin_ws/build/husky/husky_bringup/catkin_generated/installspace/husky_bringupConfig.cmake"
    "/home/tony/ugv/catkin_ws/build/husky/husky_bringup/catkin_generated/installspace/husky_bringupConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husky_bringup" TYPE FILE FILES "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/env-hooks/50.husky_find_mag_config.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/husky_bringup" TYPE PROGRAM FILES
    "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/scripts/install"
    "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/scripts/calibrate_compass"
    "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/scripts/compute_calibration"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/husky_bringup" TYPE DIRECTORY FILES
    "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/launch"
    "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/udev"
    "/home/tony/ugv/catkin_ws/src/husky/husky_bringup/config"
    )
endif()

