#ifndef BEHAVIORS__VISIBILITY_CONTROL_H_
#define BEHAVIORS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BEHAVIORS_EXPORT __attribute__ ((dllexport))
    #define BEHAVIORS_IMPORT __attribute__ ((dllimport))
  #else
    #define BEHAVIORS_EXPORT __declspec(dllexport)
    #define BEHAVIORS_IMPORT __declspec(dllimport)
  #endif
  #ifdef BEHAVIORS_BUILDING_LIBRARY
    #define BEHAVIORS_PUBLIC BEHAVIORS_EXPORT
  #else
    #define BEHAVIORS_PUBLIC BEHAVIORS_IMPORT
  #endif
  #define BEHAVIORS_PUBLIC_TYPE BEHAVIORS_PUBLIC
  #define BEHAVIORS_LOCAL
#else
  #define BEHAVIORS_EXPORT __attribute__ ((visibility("default")))
  #define BEHAVIORS_IMPORT
  #if __GNUC__ >= 4
    #define BEHAVIORS_PUBLIC __attribute__ ((visibility("default")))
    #define BEHAVIORS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BEHAVIORS_PUBLIC
    #define BEHAVIORS_LOCAL
  #endif
  #define BEHAVIORS_PUBLIC_TYPE
#endif

#endif  // BEHAVIORS__VISIBILITY_CONTROL_H_
