set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin" CACHE INTERNAL "Where to place executables and dlls")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/bin" CACHE INTERNAL "Where to place executables and dlls in release mode")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd")
set(CMAKE_MINSIZEREL_POSTFIX "_ms")

include(CMakeDependentOption)

cmake_dependent_option(USE_PYTHON_BINDINGS "Generate Python Bindings using PyBind11" ON "PYTHON_EXECUTABLE" OFF)
if (USE_PYTHON_BINDINGS AND UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    message(STATUS "Adding -fPIC option when generating Python bindings using GCC")
endif ()

option(CI_BUILD "Build on CI System" OFF)
mark_as_advanced(CI_BUILD)

if (NOT WIN32)
	if (NOT EXISTS ${PROJECT_BINARY_DIR}/CMakeCache.txt)
	  if (NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
	  endif()
	endif()
endif()

option(USE_OpenMP "Use OpenMP" ON)
if(USE_OpenMP)
	FIND_PACKAGE(OpenMP)
	if(OPENMP_FOUND)
		if (CMAKE_VERSION VERSION_GREATER "3.8")
			link_libraries(OpenMP::OpenMP_CXX)
		else()
			SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		endif()
	endif()
endif()

if (MSVC)
    set(CMAKE_USE_RELATIVE_PATHS "1")
    # Set compiler flags
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MP /bigobj")
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /MP /Ox /Ob2 /Oi /Ot /D NDEBUG")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD /Zi /MP /Ox /Ob2 /Oi /Ot /D NDEBUG /bigobj") 
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
endif (MSVC)

if (UNIX OR MINGW)
    set(CMAKE_USE_RELATIVE_PATHS "1")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
    # Set compiler flags for "release" Use generic 64 bit instructions when building on CI
	if (CI_BUILD)
		set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=x86-64")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -march=x86-64")
	elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
		set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -mcpu=native")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -mcpu=native")
	else()
		set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG")
		# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
		# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -march=native")
	endif ()
endif (UNIX OR MINGW)
if(MINGW)
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1 -Wa,-mbig-obj")
endif(MINGW)

if(APPLE)
    set(CMAKE_MACOSX_RPATH 1)
	add_definitions(-DGL_SILENCE_DEPRECATION)
endif()

set (CMAKE_CXX_STANDARD 11)

if (MSVC)
add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
endif(MSVC)

OPTION(USE_DOUBLE_PRECISION "Use double precision"	ON)
if (USE_DOUBLE_PRECISION)
	add_definitions( -DUSE_DOUBLE)	
endif (USE_DOUBLE_PRECISION)
