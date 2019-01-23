#ifndef ROBOTIS_DEVICE__VISIBILITY_CONTROL_H_
#define ROBOTIS_DEVICE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTIS_DEVICE_EXPORT __attribute__ ((dllexport))
    #define ROBOTIS_DEVICE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTIS_DEVICE_EXPORT __declspec(dllexport)
    #define ROBOTIS_DEVICE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTIS_DEVICE_BUILDING_LIBRARY
    #define ROBOTIS_DEVICE_PUBLIC ROBOTIS_DEVICE_EXPORT
  #else
    #define ROBOTIS_DEVICE_PUBLIC ROBOTIS_DEVICE_IMPORT
  #endif
  #define ROBOTIS_DEVICE_PUBLIC_TYPE ROBOTIS_DEVICE_PUBLIC
  #define ROBOTIS_DEVICE_LOCAL
#else
  #define ROBOTIS_DEVICE_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTIS_DEVICE_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTIS_DEVICE_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTIS_DEVICE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTIS_DEVICE_PUBLIC
    #define ROBOTIS_DEVICE_LOCAL
  #endif
  #define ROBOTIS_DEVICE_PUBLIC_TYPE
#endif

#endif  // ROBOTIS_DEVICE__VISIBILITY_CONTROL_H_
