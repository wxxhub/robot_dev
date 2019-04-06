#ifndef OP3_WALKING_MODULE__VISIBILITY_CONTROL_H_
#define OP3_WALKING_MODULE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OP3_WALKING_MODULE_EXPORT __attribute__ ((dllexport))
    #define OP3_WALKING_MODULE_IMPORT __attribute__ ((dllimport))
  #else
    #define OP3_WALKING_MODULE_EXPORT __declspec(dllexport)
    #define OP3_WALKING_MODULE_IMPORT __declspec(dllimport)
  #endif
  #ifdef OP3_WALKING_MODULE_BUILDING_LIBRARY
    #define OP3_WALKING_MODULE_PUBLIC OP3_WALKING_MODULE_EXPORT
  #else
    #define OP3_WALKING_MODULE_PUBLIC OP3_WALKING_MODULE_IMPORT
  #endif
  #define OP3_WALKING_MODULE_PUBLIC_TYPE OP3_WALKING_MODULE_PUBLIC
  #define OP3_WALKING_MODULE_LOCAL
#else
  #define OP3_WALKING_MODULE_EXPORT __attribute__ ((visibility("default")))
  #define OP3_WALKING_MODULE_IMPORT
  #if __GNUC__ >= 4
    #define OP3_WALKING_MODULE_PUBLIC __attribute__ ((visibility("default")))
    #define OP3_WALKING_MODULE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OP3_WALKING_MODULE_PUBLIC
    #define OP3_WALKING_MODULE_LOCAL
  #endif
  #define OP3_WALKING_MODULE_PUBLIC_TYPE
#endif

#endif  // OP3_WALKING_MODULE__VISIBILITY_CONTROL_H_
