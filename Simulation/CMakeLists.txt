add_library(Simulation
		AABB.h
		CollisionDetection.cpp
		CollisionDetection.h
		Constraints.cpp
		Constraints.h
		CubicSDFCollisionDetection.cpp
		CubicSDFCollisionDetection.h
		DistanceFieldCollisionDetection.cpp
		DistanceFieldCollisionDetection.h
		IDFactory.cpp
		IDFactory.h
		LineModel.cpp
		LineModel.h
		NeighborhoodSearchSpatialHashing.cpp
		NeighborhoodSearchSpatialHashing.h
		ParticleData.h
		RigidBody.h
		RigidBodyGeometry.cpp
		RigidBodyGeometry.h
		Simulation.cpp
		Simulation.h
		SimulationModel.cpp
		SimulationModel.h
		TetModel.cpp
		TetModel.h
		TimeManager.cpp
		TimeManager.h
		TimeStep.cpp
		TimeStep.h
		TimeStepController.cpp
		TimeStepController.h
		TriangleModel.cpp
		TriangleModel.h
		
		BoundingSphere.h
		BoundingSphereHierarchy.cpp
		BoundingSphereHierarchy.h
		kdTree.h
		kdTree.inl

		CMakeLists.txt
)

############################################################
# Discregrid
############################################################
include_directories(${ExternalInstallDir}/Discregrid/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)

############################################################
# GenericParameters
############################################################
include_directories(${ExternalInstallDir}/GenericParameters/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)

add_dependencies(Simulation ${SIMULATION_DEPENDENCIES})

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/Miniball)

target_link_libraries(Simulation PositionBasedDynamics)


install(TARGETS Simulation
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY .
DESTINATION include/Simulation
FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")