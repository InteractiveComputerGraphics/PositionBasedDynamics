cmake_minimum_required(VERSION 2.8)

PROJECT(PositionBasedDynamics)

SET(PROJECT_PATH ${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

INCLUDE(${PROJECT_PATH}/CMake/Common.cmake)
INCLUDE(${PROJECT_PATH}/CMake/AntTweakBar.cmake)      

if(WIN32)
  SUBDIRS(extern/freeglut Demos PositionBasedDynamics)
else()
  SUBDIRS(Demos PositionBasedDynamics)
endif()






