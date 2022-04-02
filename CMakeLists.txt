################################################################################
# general CMake and project setup
################################################################################
cmake_minimum_required(VERSION 3.1)

project(PositionBasedDynamics)

set(PROJECT_PATH ${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(${PROJECT_PATH}/CMake/Common.cmake)

if (NOT WIN32)
	message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

if (DEFINED PBD_EXTERNALINSTALLDIR)
	set(ExternalInstallDir "${PBD_EXTERNALINSTALLDIR}" CACHE INTERNAL "External install directory")	
else()
	set(ExternalInstallDir "${PROJECT_BINARY_DIR}" CACHE INTERNAL "External install directory")	
endif()	

include(ExternalProject)

## Eigen3 is used by most of the libraries that follow
find_package(Eigen3 REQUIRED)

set(EXT_CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	set(EXT_CMAKE_BUILD_TYPE "Release")
endif()

## Discregrid
if ((DEFINED Discregrid_INCLUDE_DIR) AND (DEFINED Discregrid_DEBUG_LIB) AND (DEFINED Discregrid_LIB))
	message(STATUS "Using ${Discregrid_INCLUDE_DIR}")
	set(Discregrid_LIBRARIES optimized ${Discregrid_LIB} debug ${Discregrid_DEBUG_LIB})
else()
	ExternalProject_Add(
	   Ext_Discregrid
	   PREFIX "${CMAKE_BINARY_DIR}/extern/Discregrid"
	   GIT_REPOSITORY https://github.com/ruberith/Discregrid.git
	   GIT_TAG "a9364dfc40617dcd1ec794f85ac32de359a11e8f"
	   INSTALL_DIR ${ExternalInstallDir}/Discregrid
	   CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${EXT_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX:PATH=${ExternalInstallDir}/Discregrid
	   -DBUILD_CMD_EXECUTABLE:BOOL=0 -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
	) 
	ExternalProject_Get_Property(Ext_Discregrid INSTALL_DIR)
	set(Discregrid_INCLUDE_DIR ${INSTALL_DIR}/include)
	set(Discregrid_LIBRARIES optimized ${INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}Discregrid${CMAKE_STATIC_LIBRARY_SUFFIX} 
							 debug ${INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}Discregrid_d${CMAKE_STATIC_LIBRARY_SUFFIX})
	unset(INSTALL_DIR)
	message(STATUS "Building ${Discregrid_INCLUDE_DIR}")
endif()

if (DEFINED GenericParameters_INCLUDE_DIR)
	message(STATUS "Using ${GenericParameters_INCLUDE_DIR}")
else()
	## GenericParameters
	ExternalProject_Add(
	   Ext_GenericParameters
	   PREFIX "${ExternalInstallDir}/GenericParameters"
	   GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/GenericParameters.git
	   GIT_TAG "9f1570aad21017c0116adf6a42bc4a19547df6ec"
	   INSTALL_DIR ${ExternalInstallDir}/GenericParameters
	   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${EXT_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX:PATH=${ExternalInstallDir}/GenericParameters -DGENERICPARAMETERS_NO_TESTS:BOOL=1		
	) 
	ExternalProject_Get_Property(Ext_GenericParameters INSTALL_DIR)
	set(GenericParameters_INCLUDE_DIR ${INSTALL_DIR}/include)
	unset(INSTALL_DIR)
endif()

add_subdirectory(PositionBasedDynamics)
add_subdirectory(Simulation)
add_subdirectory(Utils)
if (NOT PBD_LIBS_ONLY)
	include(DataCopyTargets)
	add_subdirectory(extern/glfw)
	add_subdirectory(extern/AntTweakBar)
	add_subdirectory(extern/md5)
	add_subdirectory(Demos)
	if (USE_PYTHON_BINDINGS)
		add_subdirectory(extern/pybind)
		add_subdirectory(pyPBD)
	endif ()
endif()


install(DIRECTORY ./Common
DESTINATION include
FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ./data
DESTINATION include
FILES_MATCHING PATTERN "*.glsl")

install(DIRECTORY ./Demos/Visualization
DESTINATION include
FILES_MATCHING PATTERN "*.h")

