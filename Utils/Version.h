#ifndef __Version_h__
#define __Version_h__

#define STRINGIZE_HELPER(x) #x
#define STRINGIZE(x) STRINGIZE_HELPER(x)
#define WARNING(desc) message(__FILE__ "(" STRINGIZE(__LINE__) ") : Warning: " #desc)

#define GIT_SHA1 "fc0a3e9f1c798f8ff4290a75ef543761ce23cae0"
#define GIT_REFSPEC ""
#define GIT_LOCAL_STATUS "CLEAN"

#define PBD_VERSION "2.2.0"

#ifdef DL_OUTPUT

#endif

#endif
