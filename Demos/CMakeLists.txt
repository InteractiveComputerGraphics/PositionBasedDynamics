include(Visualization/CMakeLists.txt)
add_definitions(-DPBD_DATA_PATH="../data")

# search all demos
file (
  GLOB _demo_dirs
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
           "${CMAKE_CURRENT_SOURCE_DIR}/*/CMakeLists.txt"
)

foreach (_demo_dir ${_demo_dirs})
	get_filename_component (_demo_name ${_demo_dir} PATH)
	option(Build_${_demo_name} "Build ${_demo_name}"	ON)
	if (Build_${_demo_name})
		add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/${_demo_name})
	endif (Build_${_demo_name})
endforeach ()

install(DIRECTORY ./Common
DESTINATION include/Demos
FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ./Visualization
DESTINATION include/Demos
FILES_MATCHING PATTERN "*.h")