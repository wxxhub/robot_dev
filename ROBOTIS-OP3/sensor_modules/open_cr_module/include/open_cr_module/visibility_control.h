#ifndef OPEN_CR_MODULE__VISIBILITY_CONTROL_H_
#define OPEN_CR_MODULE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OPEN_CR_MODULE_EXPORT __attribute__ ((dllexport))
    #define OPEN_CR_MODULE_IMPORT __attribute__ ((dllimport))
  #else
    #define OPEN_CR_MODULE_EXPORT __declspec(dllexport)
    #define OPEN_CR_MODULE_IMPORT __declspec(dllimport)
  #endif
  #ifdef OPEN_CR_MODULE_BUILDING_LIBRARY
    #define OPEN_CR_MODULE_PUBLIC OPEN_CR_MODULE_EXPORT
  #else
    #define OPEN_CR_MODULE_PUBLIC OPEN_CR_MODULE_IMPORT
  #endif
  #define OPEN_CR_MODULE_PUBLIC_TYPE OPEN_CR_MODULE_PUBLIC
  #define OPEN_CR_MODULE_LOCAL
#else
  #define OPEN_CR_MODULE_EXPORT __attribute__ ((visibility("default")))
  #define OPEN_CR_MODULE_IMPORT
  #if __GNUC__ >= 4
    #define OPEN_CR_MODULE_PUBLIC __attribute__ ((visibility("default")))
    #define OPEN_CR_MODULE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OPEN_CR_MODULE_PUBLIC
    #define OPEN_CR_MODULE_LOCAL
  #endif
  #define OPEN_CR_MODULE_PUBLIC_TYPE
#endif

#endif  // OPEN_CR_MODULE__VISIBILITY_CONTROL_H_
