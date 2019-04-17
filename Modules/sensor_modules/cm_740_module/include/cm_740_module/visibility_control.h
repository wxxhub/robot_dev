#ifndef CM_740_MODULE__VISIBILITY_CONTROL_H_
#define CM_740_MODULE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CM_740_MODULE_EXPORT __attribute__ ((dllexport))
    #define CM_740_MODULE_IMPORT __attribute__ ((dllimport))
  #else
    #define CM_740_MODULE_EXPORT __declspec(dllexport)
    #define CM_740_MODULE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CM_740_MODULE_BUILDING_LIBRARY
    #define CM_740_MODULE_PUBLIC CM_740_MODULE_EXPORT
  #else
    #define CM_740_MODULE_PUBLIC CM_740_MODULE_IMPORT
  #endif
  #define CM_740_MODULE_PUBLIC_TYPE CM_740_MODULE_PUBLIC
  #define CM_740_MODULE_LOCAL
#else
  #define CM_740_MODULE_EXPORT __attribute__ ((visibility("default")))
  #define CM_740_MODULE_IMPORT
  #if __GNUC__ >= 4
    #define CM_740_MODULE_PUBLIC __attribute__ ((visibility("default")))
    #define CM_740_MODULE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CM_740_MODULE_PUBLIC
    #define CM_740_MODULE_LOCAL
  #endif
  #define CM_740_MODULE_PUBLIC_TYPE
#endif

#endif  // CM_740_MODULE__VISIBILITY_CONTROL_H_
