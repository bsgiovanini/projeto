# Install script for directory: /home/robotica/bruno_workspace/src/projeto/octomap/octomap

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/octomap" TYPE FILE FILES
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTree.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/octomap_types.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeBaseSE.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/ScanGraph.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/MapCollection.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/ColorOcTree.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeDataNode.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeBase.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/MapNode.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/AbstractOcTree.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/MCTables.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/AbstractOccupancyOcTree.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeKey.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeNode.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeBaseImpl.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/octomap_deprecated.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/Pointcloud.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeStamped.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/CountingOcTree.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/octomap_timing.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/octomap_utils.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeLUT.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/octomap.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OccupancyOcTreeBase.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeLUTdefs.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeIterator.hxx"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OccupancyOcTreeBase.hxx"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeBaseImpl.hxx"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeDataNode.hxx"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/OcTreeBaseSE.hxx"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/MapCollection.hxx"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/MapNode.hxx"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/octomap/math" TYPE FILE FILES
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/math/Pose6D.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/math/Utils.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/math/Vector3.h"
    "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/include/octomap/math/Quaternion.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/octomap" TYPE FILE FILES "/home/robotica/bruno_workspace/src/projeto/octomap/octomap/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/octomap" TYPE FILE FILES
    "/home/robotica/bruno_workspace/src/projeto/octomap/build/octomap/InstallFiles/octomap-config.cmake"
    "/home/robotica/bruno_workspace/src/projeto/octomap/build/octomap/InstallFiles/octomap-config-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robotica/bruno_workspace/src/projeto/octomap/build/lib/pkgconfig/octomap.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/robotica/bruno_workspace/src/projeto/octomap/build/octomap/src/math/cmake_install.cmake")
  INCLUDE("/home/robotica/bruno_workspace/src/projeto/octomap/build/octomap/src/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

