
#ifndef cuNSearch_EXPORT_H
#define cuNSearch_EXPORT_H

#ifdef CUNSEARCH_STATIC_DEFINE
#  define cuNSearch_EXPORT
#  define CUNSEARCH_NO_EXPORT
#else
#  ifndef cuNSearch_EXPORT
#    ifdef cuNSearch_EXPORTS
        /* We are building this library */
#      define cuNSearch_EXPORT 
#    else
        /* We are using this library */
#      define cuNSearch_EXPORT 
#    endif
#  endif

#  ifndef CUNSEARCH_NO_EXPORT
#    define CUNSEARCH_NO_EXPORT 
#  endif
#endif

#ifndef CUNSEARCH_DEPRECATED
#  define CUNSEARCH_DEPRECATED __declspec(deprecated)
#endif

#ifndef CUNSEARCH_DEPRECATED_EXPORT
#  define CUNSEARCH_DEPRECATED_EXPORT cuNSearch_EXPORT CUNSEARCH_DEPRECATED
#endif

#ifndef CUNSEARCH_DEPRECATED_NO_EXPORT
#  define CUNSEARCH_DEPRECATED_NO_EXPORT CUNSEARCH_NO_EXPORT CUNSEARCH_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CUNSEARCH_NO_DEPRECATED
#    define CUNSEARCH_NO_DEPRECATED
#  endif
#endif

#endif /* cuNSearch_EXPORT_H */
