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

add_executable(FluidDemo
	  main.cpp
	  
	  TimeStepFluidModel.cpp
	  TimeStepFluidModel.h
	  FluidModel.cpp
	  FluidModel.h	  
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Demos/Utils/Config.h			
	  ${PROJECT_PATH}/Demos/Utils/Utilities.cpp
	  ${PROJECT_PATH}/Demos/Utils/Utilities.h
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.cpp	
	  ${PROJECT_PATH}/Demos/Simulation/TimeManager.h		
	  ${PROJECT_PATH}/Demos/Simulation/ParticleData.h	
	  ${PROJECT_PATH}/Demos/Simulation/NeighborhoodSearchSpatialHashing.cpp  
	  ${PROJECT_PATH}/Demos/Simulation/NeighborhoodSearchSpatialHashing.h
	  
	  CMakeLists.txt
)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)

set_target_properties(FluidDemo PROPERTIES FOLDER "Demos")
set_target_properties(FluidDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(FluidDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(FluidDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(FluidDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(FluidDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
