
add_library(freeglut SHARED
	fg_callbacks.c
	fg_cursor.c
	fg_display.c
	fg_ext.c
	fg_font.c
	fg_font_data.c
	fg_gamemode.c
	fg_geometry.c
	fg_gl2.c
	fg_gl2.h
	fg_init.c
	fg_init.h
	fg_input_devices.c
	fg_internal.h
	fg_joystick.c
	fg_main.c
	fg_menu.c
	fg_misc.c
	fg_overlay.c
	fg_spaceball.c
	fg_state.c
	fg_stroke_mono_roman.c
	fg_stroke_roman.c
	fg_structure.c
	fg_teapot.c
	fg_teapot_data.h
	#fg_version.h
	#fg_version.h.in
	fg_videoresize.c
	fg_window.c
	#freeglutdll.def.in
	#gles_stubs.c
	
	mswin/fg_cursor_mswin.c
	mswin/fg_display_mswin.c
	mswin/fg_ext_mswin.c
	mswin/fg_gamemode_mswin.c
	mswin/fg_init_mswin.c
	mswin/fg_input_devices_mswin.c
	mswin/fg_internal_mswin.h
	mswin/fg_joystick_mswin.c
	mswin/fg_main_mswin.c
	mswin/fg_menu_mswin.c
	mswin/fg_spaceball_mswin.c
	mswin/fg_state_mswin.c
	mswin/fg_structure_mswin.c
	mswin/fg_window_mswin.c

	util/xparsegeometry_repl.c
	util/xparsegeometry_repl.h
	
	../include/GL/freeglut.h
	../include/GL/freeglut_ext.h
	../include/GL/freeglut_std.h
	../include/GL/glut.h
	  
	CMakeLists.txt
)

add_definitions(-DFREEGLUT_EXPORTS)         
add_definitions(-D_USRDLL)  
add_definitions(-D_CRT_SECURE_NO_WARNINGS) 

set_target_properties(freeglut PROPERTIES FOLDER "External Dependencies")
include_directories(${PROJECT_PATH}/extern/freeglut/include)




