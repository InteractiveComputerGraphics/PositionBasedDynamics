#ifndef __CONFIG_H__
#define __CONFIG_H__

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)	   
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
#else
  #define REPORT_MEMORY_LEAKS
  #define FORCE_INLINE __attribute__((always_inline))
#endif

#endif
