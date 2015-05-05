#ifndef __CONFIG_H__
#define __CONFIG_H__

// Enable memory leak detection
#ifdef _DEBUG
	#define _CRTDBG_MAP_ALLOC 
	#include <stdlib.h> 
	#include <crtdbg.h> 
	#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__) 	
	#define REPORT_MEMORY_LEAKS _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#else
	#define REPORT_MEMORY_LEAKS
#endif

#define FORCE_INLINE __forceinline

#endif
