set(SIMULATION_LINK_LIBRARIES AntTweakBar glew PositionBasedDynamics Simulation Utils)
set(SIMULATION_DEPENDENCIES AntTweakBar glew PositionBasedDynamics Simulation Utils)

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

add_executable(FluidDemo
	  main.cpp
	  
	  TimeStepFluidModel.cpp
	  TimeStepFluidModel.h
	  FluidModel.cpp
	  FluidModel.h	  
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h
	  
	  CMakeLists.txt
)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC -DFREEGLUT_LIB_PRAGMAS=0)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)

set_target_properties(FluidDemo PROPERTIES FOLDER "Demos")
set_target_properties(FluidDemo PROPERTIES DEBUG_POSTFIX ${PBD_BINARY_DEBUG_POSTFIX})
set_target_properties(FluidDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${PBD_BINARY_RELWITHDEBINFO_POSTFIX})
set_target_properties(FluidDemo PROPERTIES MINSIZEREL_POSTFIX ${PBD_BINARY_MINSIZEREL_POSTFIX})
add_dependencies(FluidDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(FluidDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
