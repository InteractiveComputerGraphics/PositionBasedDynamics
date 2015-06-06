add_library(PositionBasedDynamics
		PositionBasedDynamics.cpp
		PositionBasedDynamics.h
		SPHKernels.cpp
		SPHKernels.h
		TimeIntegration.cpp
		TimeIntegration.h
		CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )






