#ifndef ROAD_DETECTOR__VISIBILITY_CONTROL_H_
#define ROAD_DETECTOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROAD_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define ROAD_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define ROAD_DETECTOR_EXPORT __declspec(dllexport)
    #define ROAD_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROAD_DETECTOR_BUILDING_LIBRARY
    #define ROAD_DETECTOR_PUBLIC ROAD_DETECTOR_EXPORT
  #else
    #define ROAD_DETECTOR_PUBLIC ROAD_DETECTOR_IMPORT
  #endif
  #define ROAD_DETECTOR_PUBLIC_TYPE ROAD_DETECTOR_PUBLIC
  #define ROAD_DETECTOR_LOCAL
#else
  #define ROAD_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define ROAD_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define ROAD_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define ROAD_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROAD_DETECTOR_PUBLIC
    #define ROAD_DETECTOR_LOCAL
  #endif
  #define ROAD_DETECTOR_PUBLIC_TYPE
#endif

#endif  // ROAD_DETECTOR__VISIBILITY_CONTROL_H_
