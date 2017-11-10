if (NOT PBD_NO_DEMOS)
	include_directories(include)

	add_library(glew SHARED
		src/glew.c
		include/GL/glew.h
		include/GL/glxew.h
		include/GL/wglew.h
	)

	set_target_properties(glew PROPERTIES FOLDER "External Dependencies")
	add_definitions(-DGLEW_NO_GLU -DGLEW_BUILD)

	if (WIN32)
		target_link_libraries(glew opengl32.lib)                     
	endif()

	if(APPLE)   
	   include_directories ( /System/Library/Frameworks )
	   find_library(COCOA_LIBRARY Cocoa)
	   find_library(GLUT_LIBRARY GLUT )
	   find_library(OpenGL_LIBRARY OpenGL )
	   mark_as_advanced (COCOA_LIBRARY
						 GLUT_LIBRARY
						 OpenGL_LIBRARY)
	   set(EXTRA_LIBS ${COCOA_LIBRARY} ${GLUT_LIBRARY} ${OpenGL_LIBRARY})
	   target_link_libraries(glew ${EXTRA_LIBS})
	endif (APPLE)
endif()