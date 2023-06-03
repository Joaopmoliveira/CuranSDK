# Install script for directory: D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/RBDL")
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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/Debug/rbdl.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/Release/rbdl.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/MinSizeRel/rbdl.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/RelWithDebInfo/rbdl.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rbdl" TYPE FILE FILES
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Body.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/compileassert.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Contacts.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Dynamics.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Joint.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Kinematics.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Logging.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Model.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/Quaternion.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/rbdl.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/rbdl_eigenmath.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/rbdl_math.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/rbdl_mathutils.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/rbdl_utils.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/SpatialAlgebraOperators.h"
    "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/include/rbdl/rbdl_config.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rbdl" TYPE DIRECTORY FILES "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/include/rbdl/SimpleMath")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/rbdl.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "D:/1_Projekte/2014_PhysicsEngine/RBDL/rbdl-rbdl-de94c4fadf94/myBuild/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
