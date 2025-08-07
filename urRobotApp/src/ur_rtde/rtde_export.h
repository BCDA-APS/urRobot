
#ifndef RTDE_EXPORT_H
#define RTDE_EXPORT_H

#ifdef RTDE_STATIC_DEFINE
#  define RTDE_EXPORT
#  define RTDE_NO_EXPORT
#else
#  ifndef RTDE_EXPORT
#    ifdef rtde_EXPORTS
        /* We are building this library */
#      define RTDE_EXPORT 
#    else
        /* We are using this library */
#      define RTDE_EXPORT 
#    endif
#  endif

#  ifndef RTDE_NO_EXPORT
#    define RTDE_NO_EXPORT 
#  endif
#endif

#ifndef RTDE_DEPRECATED
#  define RTDE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef RTDE_DEPRECATED_EXPORT
#  define RTDE_DEPRECATED_EXPORT RTDE_EXPORT RTDE_DEPRECATED
#endif

#ifndef RTDE_DEPRECATED_NO_EXPORT
#  define RTDE_DEPRECATED_NO_EXPORT RTDE_NO_EXPORT RTDE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef RTDE_NO_DEPRECATED
#    define RTDE_NO_DEPRECATED
#  endif
#endif

#endif /* RTDE_EXPORT_H */
