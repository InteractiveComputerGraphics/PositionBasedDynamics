#ifndef __Version_h__
#define __Version_h__

#define STRINGIZE_HELPER(x) #x
#define STRINGIZE(x) STRINGIZE_HELPER(x)
#define WARNING(desc) message(__FILE__ "(" STRINGIZE(__LINE__) ") : Warning: " #desc)

#define GIT_SHA1 "5a1aa201a12cd6c980075885520398b7f2cd259b"
#define GIT_REFSPEC "refs/heads/master"
#define GIT_LOCAL_STATUS "DIRTY"

#define PBD_VERSION "2.2.0"

#ifdef DL_OUTPUT
#pragma WARNING(Local changes not committed.)
#endif

#endif
