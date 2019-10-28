set(SIMULATION_LINK_LIBRARIES AntTweakBar glew MD5 PositionBasedDynamics Simulation Utils)
set(SIMULATION_DEPENDENCIES AntTweakBar glew MD5 PositionBasedDynamics Simulation Utils)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES freeglut opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
  set(SIMULATION_DEPENDENCIES freeglut ${SIMULATION_DEPENDENCIES})
else()
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(GLUT REQUIRED)
  find_package(OpenGL REQUIRED)

  set(SIMULATION_LINK_LIBRARIES
    ${SIMULATION_LINK_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
endif()

############################################################
# Discregrid
############################################################
include_directories(${PROJECT_BINARY_DIR}/Discregrid/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} 
	optimized Discregrid 
	debug Discregrid_d)
link_directories(${PROJECT_BINARY_DIR}/Discregrid/lib)

############################################################
# GenericParameters
############################################################
include_directories(${PROJECT_BINARY_DIR}/GenericParameters/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)

add_executable(SceneLoaderDemo
	  SceneLoaderDemo.cpp

	  ../Common/TweakBarParameters.cpp
	  ../Common/TweakBarParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(SceneLoaderDemo PROPERTIES FOLDER "Demos")
set_target_properties(SceneLoaderDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(SceneLoaderDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(SceneLoaderDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(SceneLoaderDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(SceneLoaderDemo ${SIMULATION_LINK_LIBRARIES})


add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC -DFREEGLUT_LIB_PRAGMAS=0)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)


VIS_SOURCE_GROUPS()
