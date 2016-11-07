add_library(Utils
		Hashmap.h
		IndexedFaceMesh.cpp
		IndexedFaceMesh.h
		IndexedTetMesh.cpp
		IndexedTetMesh.h
		ObjectArray.h
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

install(TARGETS Utils
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY .
DESTINATION include/Demos/Utils
FILES_MATCHING PATTERN "*.h")




