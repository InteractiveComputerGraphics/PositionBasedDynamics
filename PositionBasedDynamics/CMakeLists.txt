add_library(PositionBasedDynamics
		PositionBasedDynamics.cpp
		PositionBasedDynamics.h
		TimeIntegration.cpp
		TimeIntegration.h
		CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

set_target_properties(PositionBasedDynamics PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE ${IBDS_STATIC_LIBRARY_FLAGS_RELEASE})   





