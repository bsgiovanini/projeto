# Install script for directory: /home/bruno/octomap/octomap

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
    "/home/bruno/octomap/octomap/include/octomap/ColorOcTree.h"
    "/home/bruno/octomap/octomap/include/octomap/MCTables.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeKey.h"
    "/home/bruno/octomap/octomap/include/octomap/AbstractOcTree.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeNode.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeBaseSE.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTree.h"
    "/home/bruno/octomap/octomap/include/octomap/octomap.h"
    "/home/bruno/octomap/octomap/include/octomap/octomap_types.h"
    "/home/bruno/octomap/octomap/include/octomap/ScanGraph.h"
    "/home/bruno/octomap/octomap/include/octomap/CountingOcTree.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeBase.h"
    "/home/bruno/octomap/octomap/include/octomap/AbstractOccupancyOcTree.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeDataNode.h"
    "/home/bruno/octomap/octomap/include/octomap/OccupancyOcTreeBase.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeLUT.h"
    "/home/bruno/octomap/octomap/include/octomap/octomap_utils.h"
    "/home/bruno/octomap/octomap/include/octomap/octomap_timing.h"
    "/home/bruno/octomap/octomap/include/octomap/MapNode.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeBaseImpl.h"
    "/home/bruno/octomap/octomap/include/octomap/octomap_deprecated.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeStamped.h"
    "/home/bruno/octomap/octomap/include/octomap/MapCollection.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeLUTdefs.h"
    "/home/bruno/octomap/octomap/include/octomap/Pointcloud.h"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeBaseImpl.hxx"
    "/home/bruno/octomap/octomap/include/octomap/OccupancyOcTreeBase.hxx"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeIterator.hxx"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeDataNode.hxx"
    "/home/bruno/octomap/octomap/include/octomap/OcTreeBaseSE.hxx"
    "/home/bruno/octomap/octomap/include/octomap/MapCollection.hxx"
    "/home/bruno/octomap/octomap/include/octomap/MapNode.hxx"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/octomap/math" TYPE FILE FILES
    "/home/bruno/octomap/octomap/include/octomap/math/Utils.h"
    "/home/bruno/octomap/octomap/include/octomap/math/Pose6D.h"
    "/home/bruno/octomap/octomap/include/octomap/math/Quaternion.h"
    "/home/bruno/octomap/octomap/include/octomap/math/Vector3.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/octomap" TYPE FILE FILES "/home/bruno/octomap/octomap/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/octomap" TYPE FILE FILES
    "/home/bruno/octomap/build/octomap/InstallFiles/octomap-config.cmake"
    "/home/bruno/octomap/build/octomap/InstallFiles/octomap-config-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bruno/octomap/build/lib/pkgconfig/octomap.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bruno/octomap/build/octomap/src/math/cmake_install.cmake")
  INCLUDE("/home/bruno/octomap/build/octomap/src/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

