set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd")
set(CMAKE_MINSIZEREL_POSTFIX "_ms")

if (WIN32)
    set(CMAKE_USE_RELATIVE_PATHS "1")
    # Set compiler flags
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MP")
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /MP /Ox /Ob2 /Oi /Ot /D NDEBUG /openmp")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
endif (WIN32)

if (UNIX)
    set(CMAKE_USE_RELATIVE_PATHS "1")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
    # Set compiler flags for "release"
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fopenmp")
endif (UNIX)

if(APPLE)
    set(CMAKE_USE_RELATIVE_PATHS "1")
    set(CMAKE_MACOSX_RPATH 1)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
    # Set compiler flags for "release"
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -Xpreprocessor -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-lomp")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "-lomp")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "-lomp")
endif()

set (CMAKE_CXX_STANDARD 11)

add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
