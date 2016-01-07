set(SIMULATION_LINK_LIBRARIES AntTweakBar glew PositionBasedDynamics)
set(SIMULATION_DEPENDENCIES AntTweakBar glew PositionBasedDynamics)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES freeglut opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
  set(SIMULATION_DEPENDENCIES freeglut ${SIMULATION_DEPENDENCIES})
else()
  find_package(GLUT REQUIRED)
  find_package(OpenGL REQUIRED)

  set(SIMULATION_LINK_LIBRARIES
    ${SIMULATION_LINK_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
endif()

add_executable(GenericParticleConstraintsDemo
	  GenericParticleConstraintsDemo.cpp
	  
	  GenericConstraintsModel.cpp
	  GenericConstraintsModel.h	  
	  GenericConstraints.cpp
	  GenericConstraints.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
  	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h		  	  
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Simulation/RigidBody.h		  
  	  ${PROJECT_PATH}/Demos/Simulation/Constraints.cpp
	  ${PROJECT_PATH}/Demos/Simulation/Constraints.h
	  ${PROJECT_PATH}/Demos/Simulation/TetModel.cpp
	  ${PROJECT_PATH}/Demos/Simulation/TetModel.h
	  ${PROJECT_PATH}/Demos/Simulation/TriangleModel.cpp
	  ${PROJECT_PATH}/Demos/Simulation/TriangleModel.h
	  ${PROJECT_PATH}/Demos/Simulation/TimeStepController.cpp
	  ${PROJECT_PATH}/Demos/Simulation/TimeStepController.h
	  ${PROJECT_PATH}/Demos/Simulation/SimulationModel.cpp
	  ${PROJECT_PATH}/Demos/Simulation/SimulationModel.h	  			  
	  
	  CMakeLists.txt
)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)

set_target_properties(GenericParticleConstraintsDemo PROPERTIES FOLDER "Demos")
add_dependencies(GenericParticleConstraintsDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(GenericParticleConstraintsDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
