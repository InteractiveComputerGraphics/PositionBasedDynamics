include_directories(${PROJECT_PATH}/extern/freeglut/include)

set(VIS_HEADER_FILES
	${PROJECT_PATH}/Demos/Visualization/MiniGL.h
	${PROJECT_PATH}/Demos/Visualization/Selection.h
	${PROJECT_PATH}/Demos/Visualization/Shader.h
	${PROJECT_PATH}/Demos/Visualization/Visualization.h
	) 

	
set(VIS_SOURCE_FILES
	${PROJECT_PATH}/Demos/Visualization/MiniGL.cpp                       
	${PROJECT_PATH}/Demos/Visualization/Shader.cpp
	) 
	

set(VIS_FILES
	${VIS_HEADER_FILES}
	${VIS_SOURCE_FILES}
	) 
	
# A macro for the source groups
macro(VIS_SOURCE_GROUPS)
	source_group("Header Files\\Visualization" FILES ${VIS_HEADER_FILES})
	source_group("Source Files\\Visualization" FILES ${VIS_SOURCE_FILES})
endmacro(VIS_SOURCE_GROUPS)



