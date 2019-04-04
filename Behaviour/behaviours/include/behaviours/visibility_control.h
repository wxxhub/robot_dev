#ifndef BEHAVIOURS__VISIBILITY_CONTROL_H_
#define BEHAVIOURS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BEHAVIOURS_EXPORT __attribute__ ((dllexport))
    #define BEHAVIOURS_IMPORT __attribute__ ((dllimport))
  #else
    #define BEHAVIOURS_EXPORT __declspec(dllexport)
    #define BEHAVIOURS_IMPORT __declspec(dllimport)
  #endif
  #ifdef BEHAVIOURS_BUILDING_LIBRARY
    #define BEHAVIOURS_PUBLIC BEHAVIOURS_EXPORT
  #else
    #define BEHAVIOURS_PUBLIC BEHAVIOURS_IMPORT
  #endif
  #define BEHAVIOURS_PUBLIC_TYPE BEHAVIOURS_PUBLIC
  #define BEHAVIOURS_LOCAL
#else
  #define BEHAVIOURS_EXPORT __attribute__ ((visibility("default")))
  #define BEHAVIOURS_IMPORT
  #if __GNUC__ >= 4
    #define BEHAVIOURS_PUBLIC __attribute__ ((visibility("default")))
    #define BEHAVIOURS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BEHAVIOURS_PUBLIC
    #define BEHAVIOURS_LOCAL
  #endif
  #define BEHAVIOURS_PUBLIC_TYPE
#endif

#endif  // BEHAVIOURS__VISIBILITY_CONTROL_H_
