SET(SIMULATION_LINK_LIBRARIES PositionBasedDynamics)
SET (SIMULATION_DEPENDENCIES PositionBasedDynamics)

if(WIN32)
  SET(SIMULATION_LINK_LIBRARIES freeglut opengl32.lib glu32.lib ${ANTTWEAKBAR_LIBS} ${SIMULATION_LINK_LIBRARIES})
  SET (SIMULATION_DEPENDENCIES freeglut ${SIMULATION_DEPENDENCIES})
endif()

ADD_EXECUTABLE(BarDemo
	  main.cpp
	  
	  TimeStepTetModel.cpp
	  TimeStepTetModel.h
	  TetModel.cpp
	  TetModel.h	  
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h
	  ${PROJECT_PATH}/Demos/Utils/ParticleData.h		  
	  
	  CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${PROJECT_PATH}/extern/freeglut/include)

if(NOT WIN32)
  find_package(GLUT REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(AntTweakBar REQUIRED)

  set(SIMULATION_LINK_LIBRARIES
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${ANTTWEAKBAR_LIBRARIES}
    ${SIMULATION_LINK_LIBRARIES}
    )
endif()

ADD_DEPENDENCIES(BarDemo ${SIMULATION_DEPENDENCIES})
TARGET_LINK_LIBRARIES(BarDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
if(WIN32)
  COPY_ANTTWEAKBAR_DLL(BarDemo)
endif()
