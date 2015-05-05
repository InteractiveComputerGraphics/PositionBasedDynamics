SET(SIMULATION_LINK_LIBRARIES freeglut PositionBasedDynamics opengl32.lib glu32.lib ${ANTTWEAKBAR_LIBS})
SET (SIMULATION_DEPENDENCIES freeglut PositionBasedDynamics)

ADD_EXECUTABLE(ClothDemo
	  main.cpp
	  
	  TimeStepTriangleModel.cpp
	  TimeStepTriangleModel.h
	  TriangleModel.cpp
	  TriangleModel.h	  
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h
	  ${PROJECT_PATH}/Demos/Utils/ParticleData.h		  
	  
	  CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${PROJECT_PATH}/extern/freeglut/include)

ADD_DEPENDENCIES(ClothDemo ${SIMULATION_DEPENDENCIES})
TARGET_LINK_LIBRARIES(ClothDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
COPY_ANTTWEAKBAR_DLL(ClothDemo)
