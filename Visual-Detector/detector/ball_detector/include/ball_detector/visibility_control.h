#ifndef BALL_DETECTOR__VISIBILITY_CONTROL_H_
#define BALL_DETECTOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BALL_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define BALL_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define BALL_DETECTOR_EXPORT __declspec(dllexport)
    #define BALL_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef BALL_DETECTOR_BUILDING_LIBRARY
    #define BALL_DETECTOR_PUBLIC BALL_DETECTOR_EXPORT
  #else
    #define BALL_DETECTOR_PUBLIC BALL_DETECTOR_IMPORT
  #endif
  #define BALL_DETECTOR_PUBLIC_TYPE BALL_DETECTOR_PUBLIC
  #define BALL_DETECTOR_LOCAL
#else
  #define BALL_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define BALL_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define BALL_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define BALL_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BALL_DETECTOR_PUBLIC
    #define BALL_DETECTOR_LOCAL
  #endif
  #define BALL_DETECTOR_PUBLIC_TYPE
#endif

#endif  // BALL_DETECTOR__VISIBILITY_CONTROL_H_
