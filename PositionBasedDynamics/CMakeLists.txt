ADD_LIBRARY(PositionBasedDynamics
		PositionBasedDynamics.cpp
		PositionBasedDynamics.h
		CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

SET_TARGET_PROPERTIES(PositionBasedDynamics PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE ${IBDS_STATIC_LIBRARY_FLAGS_RELEASE})   





