add_custom_target(CopyPBDShaders
	${CMAKE_COMMAND} -E copy_directory
	${PROJECT_SOURCE_DIR}/data/shaders
	${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources/shaders
	COMMENT "Copying PBD shaders"
)
set_target_properties(CopyPBDShaders PROPERTIES FOLDER "Data copy")

add_custom_target(CopyPBDModels
	${CMAKE_COMMAND} -E copy_directory
	${PROJECT_SOURCE_DIR}/data/models
	${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources/models
	COMMENT "Copying PBD models"
)
set_target_properties(CopyPBDModels PROPERTIES FOLDER "Data copy")

add_custom_target(CopyPBDScenes
	${CMAKE_COMMAND} -E copy_directory
	${PROJECT_SOURCE_DIR}/data/scenes
	${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources/scenes
	COMMENT "Copying PBD scenes"
)
set_target_properties(CopyPBDScenes PROPERTIES FOLDER "Data copy")
