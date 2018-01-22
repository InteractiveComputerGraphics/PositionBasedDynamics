add_library(PositionBasedDynamics
		 ${PROJECT_PATH}/Common/Common.h
		
		MathFunctions.cpp
		MathFunctions.h
		PositionBasedDynamics.cpp
		PositionBasedDynamics.h
		PositionBasedElasticRods.cpp
		PositionBasedElasticRods.h
		PositionBasedFluids.cpp
		PositionBasedFluids.h
		PositionBasedRigidBodyDynamics.cpp
		PositionBasedRigidBodyDynamics.h
		PositionBasedGenericConstraints.h
		SPHKernels.cpp
		SPHKernels.h
		TimeIntegration.cpp
		TimeIntegration.h
		
		CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
if(APPLE)
	target_link_libraries(PositionBasedDynamics omp)
endif()
include_directories( ${EIGEN3_INCLUDE_DIR} )

install(TARGETS PositionBasedDynamics
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY .
DESTINATION include/PositionBasedDynamics
FILES_MATCHING PATTERN "*.h")




