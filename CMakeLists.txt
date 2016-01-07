cmake_minimum_required(VERSION 2.8)

project(PositionBasedDynamics)

set(PROJECT_PATH ${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(${PROJECT_PATH}/CMake/Common.cmake)

if (WIN32)
  subdirs(extern/freeglut extern/AntTweakBar extern/glew Demos PositionBasedDynamics)
else()
  subdirs(extern/AntTweakBar extern/glew Demos PositionBasedDynamics)
endif()



