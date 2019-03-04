#ifndef qrcode_DETECTOR__VISIBILITY_CONTROL_H_
#define qrcode_DETECTOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define qrcode_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define qrcode_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define qrcode_DETECTOR_EXPORT __declspec(dllexport)
    #define qrcode_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef qrcode_DETECTOR_BUILDING_LIBRARY
    #define qrcode_DETECTOR_PUBLIC qrcode_DETECTOR_EXPORT
  #else
    #define qrcode_DETECTOR_PUBLIC qrcode_DETECTOR_IMPORT
  #endif
  #define qrcode_DETECTOR_PUBLIC_TYPE qrcode_DETECTOR_PUBLIC
  #define qrcode_DETECTOR_LOCAL
#else
  #define qrcode_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define qrcode_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define qrcode_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define qrcode_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define qrcode_DETECTOR_PUBLIC
    #define qrcode_DETECTOR_LOCAL
  #endif
  #define qrcode_DETECTOR_PUBLIC_TYPE
#endif

#endif  // qrcode_DETECTOR__VISIBILITY_CONTROL_H_
