#ifndef HAND_REGULATOR_MODULE__VISIBILITY_CONTROL_H_
#define HAND_REGULATOR_MODULE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HAND_REGULATOR_MODULE_EXPORT __attribute__ ((dllexport))
    #define HAND_REGULATOR_MODULE_IMPORT __attribute__ ((dllimport))
  #else
    #define HAND_REGULATOR_MODULE_EXPORT __declspec(dllexport)
    #define HAND_REGULATOR_MODULE_IMPORT __declspec(dllimport)
  #endif
  #ifdef HAND_REGULATOR_MODULE_BUILDING_LIBRARY
    #define HAND_REGULATOR_MODULE_PUBLIC HAND_REGULATOR_MODULE_EXPORT
  #else
    #define HAND_REGULATOR_MODULE_PUBLIC HAND_REGULATOR_MODULE_IMPORT
  #endif
  #define HAND_REGULATOR_MODULE_PUBLIC_TYPE HAND_REGULATOR_MODULE_PUBLIC
  #define HAND_REGULATOR_MODULE_LOCAL
#else
  #define HAND_REGULATOR_MODULE_EXPORT __attribute__ ((visibility("default")))
  #define HAND_REGULATOR_MODULE_IMPORT
  #if __GNUC__ >= 4
    #define HAND_REGULATOR_MODULE_PUBLIC __attribute__ ((visibility("default")))
    #define HAND_REGULATOR_MODULE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HAND_REGULATOR_MODULE_PUBLIC
    #define HAND_REGULATOR_MODULE_LOCAL
  #endif
  #define HAND_REGULATOR_MODULE_PUBLIC_TYPE
#endif

#endif  // HAND_REGULATOR_MODULE__VISIBILITY_CONTROL_H_
