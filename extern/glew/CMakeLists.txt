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
