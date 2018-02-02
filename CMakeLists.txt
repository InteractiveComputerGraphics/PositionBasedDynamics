cmake_minimum_required(VERSION 2.8)

project(PositionBasedDynamics)

set(PROJECT_PATH ${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(${PROJECT_PATH}/CMake/Common.cmake)

if (WIN32)
  subdirs(extern/freeglut extern/AntTweakBar extern/glew extern/md5 Demos PositionBasedDynamics)
else()
  subdirs(extern/AntTweakBar extern/glew extern/md5 Demos PositionBasedDynamics)
endif()

include(ExternalProject)
if (DEFINED PBD_EXTERNALINSTALLDIR)
	set(ExternalInstallDir "${PBD_EXTERNALINSTALLDIR}")	
else()
	set(ExternalInstallDir "${CMAKE_SOURCE_DIR}/extern/install")
endif()	
set(EigenDir "${CMAKE_SOURCE_DIR}/extern/eigen")

## Discregrid
ExternalProject_Add(
   ExternalProject_Discregrid
   PREFIX "${CMAKE_SOURCE_DIR}/extern/Discregrid"
   GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/Discregrid.git
   GIT_TAG "f16a29afebf7a7f43139d5832bbfc7124c5d98db"
   INSTALL_DIR ${ExternalInstallDir}/Discregrid
   CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${ExternalInstallDir}/Discregrid -DBUILD_CMD_EXECUTABLE:BOOL=0 -DEIGEN3_INCLUDE_DIR:PATH=${EigenDir}
) 

install(DIRECTORY ./Common
DESTINATION include
FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ./data
DESTINATION include
FILES_MATCHING PATTERN "*.glsl")

