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
		SimulationModel.cpp
		SimulationModel.h
		TetModel.cpp
		TetModel.h
		TimeManager.cpp
		TimeManager.h
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
include_directories(${PROJECT_PATH}/extern/install/Discregrid/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} ExternalProject_Discregrid)
add_dependencies(Simulation ${SIMULATION_DEPENDENCIES})

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/Miniball)

set(SIMULATION_LIBRARIES PositionBasedDynamics)
if(APPLE)
	set(SIMULATION_LIBRARIES ${SIMULATION_LIBRARIES} omp)
endif()
target_link_libraries(Simulation ${SIMULATION_LIBRARIES})


install(TARGETS Simulation
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY .
DESTINATION include/Demos/Simulation
FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")