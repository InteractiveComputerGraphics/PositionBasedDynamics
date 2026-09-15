set(IMGUI_HEADERS
	imconfig.h
	imgui.h
	imgui_internal.h
	imstb_rectpack.h
	imstb_textedit.h
	imstb_truetype.h
	
	backends/imgui_impl_glfw.h
	backends/imgui_impl_opengl2.h
)

set(IMGUI_SOURCES
	imgui.cpp
	imgui_demo.cpp # useful examples of how to use imgui
	imgui_draw.cpp
	imgui_tables.cpp
	imgui_widgets.cpp
	
	backends/imgui_impl_glfw.cpp
	backends/imgui_impl_opengl2.cpp
)

add_library(imgui
	${IMGUI_HEADERS}
	${IMGUI_SOURCES}
)

target_compile_definitions(imgui PUBLIC -DIMGUI_IMPL_OPENGL_LOADER_GLAD)

add_dependencies(imgui glfw)
target_link_libraries(imgui PUBLIC glfw)

target_include_directories(glfw PUBLIC ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(imgui PROPERTIES FOLDER "External Dependencies")

