SET(SIMULATION_LINK_LIBRARIES freeglut PositionBasedDynamics opengl32.lib glu32.lib ${ANTTWEAKBAR_LIBS})
SET (SIMULATION_DEPENDENCIES freeglut PositionBasedDynamics)

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

ADD_DEPENDENCIES(BarDemo ${SIMULATION_DEPENDENCIES})
TARGET_LINK_LIBRARIES(BarDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
COPY_ANTTWEAKBAR_DLL(BarDemo)
