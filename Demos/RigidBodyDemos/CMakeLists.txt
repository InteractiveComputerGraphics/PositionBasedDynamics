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

add_executable(ChainDemo
	  ChainDemo.cpp
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.cpp
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.h	
	  ${PROJECT_PATH}/Demos/Utils/Utilities.cpp
	  ${PROJECT_PATH}/Demos/Utils/Utilities.h		    
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.cpp
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.h		    
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Simulation/RigidBody.h	
	  ${PROJECT_PATH}/Demos/Simulation/RigidBodyGeometry.cpp
	  ${PROJECT_PATH}/Demos/Simulation/RigidBodyGeometry.h		  	    
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

set_target_properties(ChainDemo PROPERTIES FOLDER "Demos")
set_target_properties(ChainDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ChainDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ChainDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(ChainDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(ChainDemo ${SIMULATION_LINK_LIBRARIES})

add_executable(JointDemo
	  JointDemo.cpp
	    
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h		
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.cpp
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.h	
	  ${PROJECT_PATH}/Demos/Utils/Utilities.cpp
	  ${PROJECT_PATH}/Demos/Utils/Utilities.h		    
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.cpp
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.h		  	  
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Simulation/RigidBody.h		
	  ${PROJECT_PATH}/Demos/Simulation/RigidBodyGeometry.cpp
	  ${PROJECT_PATH}/Demos/Simulation/RigidBodyGeometry.h		  	  	  
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

set_target_properties(JointDemo PROPERTIES FOLDER "Demos")
set_target_properties(JointDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(JointDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(JointDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(JointDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(JointDemo ${SIMULATION_LINK_LIBRARIES})

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)


VIS_SOURCE_GROUPS()
