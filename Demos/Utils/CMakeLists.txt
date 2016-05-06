add_library(Utils
		Hashmap.h
		IndexedFaceMesh.cpp
		IndexedFaceMesh.h
		IndexedTetMesh.cpp
		IndexedTetMesh.h
		ObjectArray.h
		OBJExporter.cpp
		OBJExporter.h
		OBJLoader.cpp
		OBJLoader.h
		SceneLoader.cpp
		SceneLoader.h
		TetGenLoader.cpp
		TetGenLoader.h
		Timing.cpp
		Timing.h
		Utilities.cpp
		Utilities.h
		VolumeIntegration.cpp
		VolumeIntegration.h

		CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )






