include(Visualization/CMakeLists.txt)

subdirs(BarDemo ClothDemo ElasticRodDemos FluidDemo RigidBodyDemos CouplingDemos GenericConstraintsDemos)

# A macro for copying the resources of a sample project
MACRO(COPY_RESOURCES demoName)
	IF (WIN32)
		ADD_CUSTOM_COMMAND(
		   TARGET ${demoName}
		   POST_BUILD
		   COMMAND xcopy /r /y \"${CMAKE_CURRENT_SOURCE_DIR}/Resources_${demoName}\" \"$\(TargetDir\)..\\..\\bin\\$\(ConfigurationName\)\\Resources_${demoName}\\\"
		)                        
	ENDIF (WIN32)
ENDMACRO(COPY_RESOURCES)