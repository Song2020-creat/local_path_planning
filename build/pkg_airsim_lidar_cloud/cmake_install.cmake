# Install script for directory: /home/nvidia/airsim_ros_Astar_RRT/src/pkg_airsim_lidar_cloud

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/airsim_ros_Astar_RRT/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nvidia/airsim_ros_Astar_RRT/build/pkg_airsim_lidar_cloud/catkin_generated/installspace/pkg_airsim_lidar_cloud.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pkg_airsim_lidar_cloud/cmake" TYPE FILE FILES
    "/home/nvidia/airsim_ros_Astar_RRT/build/pkg_airsim_lidar_cloud/catkin_generated/installspace/pkg_airsim_lidar_cloudConfig.cmake"
    "/home/nvidia/airsim_ros_Astar_RRT/build/pkg_airsim_lidar_cloud/catkin_generated/installspace/pkg_airsim_lidar_cloudConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pkg_airsim_lidar_cloud" TYPE FILE FILES "/home/nvidia/airsim_ros_Astar_RRT/src/pkg_airsim_lidar_cloud/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkg_airsim_lidar_cloud" TYPE PROGRAM FILES "/home/nvidia/airsim_ros_Astar_RRT/build/pkg_airsim_lidar_cloud/catkin_generated/installspace/node_orin_cloud_publish.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkg_airsim_lidar_cloud" TYPE PROGRAM FILES "/home/nvidia/airsim_ros_Astar_RRT/build/pkg_airsim_lidar_cloud/catkin_generated/installspace/node_filtered_cloud_publish.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkg_airsim_lidar_cloud" TYPE PROGRAM FILES "/home/nvidia/airsim_ros_Astar_RRT/build/pkg_airsim_lidar_cloud/catkin_generated/installspace/node_filtered_cloud_sync_publish.py")
endif()

