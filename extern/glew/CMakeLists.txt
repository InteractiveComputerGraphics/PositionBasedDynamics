include_directories(include)

add_library(glew SHARED
    src/glew.c
	include/GL/glew.h
	include/GL/glxew.h
	include/GL/wglew.h
)

add_definitions(-DGLEW_NO_GLU -DGLEW_BUILD)

if (WIN32)
	target_link_libraries(glew opengl32.lib)

	add_custom_command(
	   TARGET glew
	   POST_BUILD
	   COMMAND xcopy /r /y \"$\(TargetPath\)" \"$\(TargetDir\)..\\..\\bin\\$\(ConfigurationName\)\\"
	)                        
endif()

IF(APPLE)   
   INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
   FIND_LIBRARY(COCOA_LIBRARY Cocoa)
   FIND_LIBRARY(GLUT_LIBRARY GLUT )
   FIND_LIBRARY(OpenGL_LIBRARY OpenGL )
   MARK_AS_ADVANCED (COCOA_LIBRARY
                     GLUT_LIBRARY
                     OpenGL_LIBRARY)
   SET(EXTRA_LIBS ${COCOA_LIBRARY} ${GLUT_LIBRARY} ${OpenGL_LIBRARY})
   target_link_libraries(glew ${EXTRA_LIBS})
ENDIF (APPLE)
