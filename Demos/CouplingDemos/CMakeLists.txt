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

add_executable(RigidBodyClothCouplingDemo
	  RigidBodyClothCouplingDemo.cpp
	  
	  TimeStepRigidBodyParticleModel.cpp
	  TimeStepRigidBodyParticleModel.h
	  RigidBodyParticleModel.cpp
	  RigidBodyParticleModel.h	  
	    
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Utils/RigidBody.h		  
	  
	  CMakeLists.txt
)

set_target_properties(RigidBodyClothCouplingDemo PROPERTIES FOLDER "Demos")
add_dependencies(RigidBodyClothCouplingDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(RigidBodyClothCouplingDemo ${SIMULATION_LINK_LIBRARIES})


add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)


VIS_SOURCE_GROUPS()
