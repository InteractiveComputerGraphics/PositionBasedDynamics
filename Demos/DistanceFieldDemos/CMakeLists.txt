set(SIMULATION_LINK_LIBRARIES AntTweakBar glew PositionBasedDynamics Simulation)
set(SIMULATION_DEPENDENCIES AntTweakBar glew PositionBasedDynamics Simulation)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES freeglut opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES} ${Boost_LIBRARIES})
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

add_executable(ClothCollisionDemo
	  ClothCollisionDemo.cpp
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h
	  ${PROJECT_PATH}/Demos/Common/Config.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.cpp
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.h	
	  ${PROJECT_PATH}/Demos/Utils/ObjectArray.h	
	  ${PROJECT_PATH}/Demos/Utils/Utilities.cpp
	  ${PROJECT_PATH}/Demos/Utils/Utilities.h		    
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.cpp
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.h		

	  CMakeLists.txt
)

set_target_properties(ClothCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(ClothCollisionDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ClothCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ClothCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(ClothCollisionDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(ClothCollisionDemo ${SIMULATION_LINK_LIBRARIES})


add_executable(DeformableCollisionDemo
	  DeformableCollisionDemo.cpp
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h
	  ${PROJECT_PATH}/Demos/Common/Config.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.cpp
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.h	
	  ${PROJECT_PATH}/Demos/Utils/ObjectArray.h	
	  ${PROJECT_PATH}/Demos/Utils/Utilities.cpp
	  ${PROJECT_PATH}/Demos/Utils/Utilities.h		    
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.cpp
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.h		

	  CMakeLists.txt
)

set_target_properties(DeformableCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(DeformableCollisionDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(DeformableCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(DeformableCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(DeformableCollisionDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(DeformableCollisionDemo ${SIMULATION_LINK_LIBRARIES})


add_executable(RigidBodyCollisionDemo
	  RigidBodyCollisionDemo.cpp
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h
	  ${PROJECT_PATH}/Demos/Common/Config.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedFaceMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.cpp
	  ${PROJECT_PATH}/Demos/Utils/IndexedTetMesh.h	
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.cpp
	  ${PROJECT_PATH}/Demos/Utils/OBJLoader.h	
	  ${PROJECT_PATH}/Demos/Utils/ObjectArray.h	
	  ${PROJECT_PATH}/Demos/Utils/Utilities.cpp
	  ${PROJECT_PATH}/Demos/Utils/Utilities.h		    
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.cpp
	  ${PROJECT_PATH}/Demos/Utils/VolumeIntegration.h		

	  CMakeLists.txt
)

set_target_properties(RigidBodyCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(RigidBodyCollisionDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(RigidBodyCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(RigidBodyCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(RigidBodyCollisionDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(RigidBodyCollisionDemo ${SIMULATION_LINK_LIBRARIES})


add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)


VIS_SOURCE_GROUPS()
