set(SIMULATION_LINK_LIBRARIES AntTweakBar glew PositionBasedDynamics)
set(SIMULATION_DEPENDENCIES AntTweakBar glew PositionBasedDynamics)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES freeglut opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
  set(SIMULATION_DEPENDENCIES freeglut ${SIMULATION_DEPENDENCIES})
else()
  find_package(GLUT REQUIRED)
  find_package(OpenGL REQUIRED)

  set(SIMULATION_LINK_LIBRARIES
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${SIMULATION_LINK_LIBRARIES}
  )
endif()

add_executable(FluidDemo
	  main.cpp
	  
	  TimeStepFluidModel.cpp
	  TimeStepFluidModel.h
	  FluidModel.cpp
	  FluidModel.h	  
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Utils/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Utils/ParticleData.h	
	  ${PROJECT_PATH}/Demos/Utils/NeighborhoodSearchSpatialHashing.cpp  
	  ${PROJECT_PATH}/Demos/Utils/NeighborhoodSearchSpatialHashing.h
	  
	  CMakeLists.txt
)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)

add_dependencies(FluidDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(FluidDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
