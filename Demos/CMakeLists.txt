include(Visualization/CMakeLists.txt)

# search all demos
set(PBD_DEMOS 
	BarDemo
	ClothDemo
	CosseratRodsDemo
	CouplingDemos
	DistanceFieldDemos
	FluidDemo
	GenericConstraintsDemos
	PositionBasedElasticRodsDemo
	RigidBodyDemos
	SceneLoaderDemo
	StiffRodsDemos
)

if (NOT PBD_NO_DEMOS)
	foreach (_demo_name ${PBD_DEMOS})
		option(Build_${_demo_name} "Build ${_demo_name}"	ON)
		if (Build_${_demo_name})
			add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/${_demo_name})
		endif (Build_${_demo_name})
	endforeach ()
endif()

install(DIRECTORY ./Common
DESTINATION include/Demos
FILES_MATCHING PATTERN "*.h")

