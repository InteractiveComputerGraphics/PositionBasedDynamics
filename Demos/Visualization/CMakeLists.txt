INCLUDE_DIRECTORIES(${PROJECT_PATH}/extern/freeglut/include)

SET(VIS_HEADER_FILES
	${PROJECT_PATH}/Demos/Visualization/MiniGL.h
	${PROJECT_PATH}/Demos/Visualization/Selection.h
	) 

	
SET(VIS_SOURCE_FILES
	${PROJECT_PATH}/Demos/Visualization/MiniGL.cpp                       
	) 
	

SET(VIS_FILES
	${VIS_HEADER_FILES}
	${VIS_SOURCE_FILES}
	) 
	
# A macro for the source groups
MACRO(VIS_SOURCE_GROUPS)
	SOURCE_GROUP("Header Files\\Visualization" FILES ${VIS_HEADER_FILES})
	SOURCE_GROUP("Source Files\\Visualization" FILES ${VIS_SOURCE_FILES})
ENDMACRO(VIS_SOURCE_GROUPS)



