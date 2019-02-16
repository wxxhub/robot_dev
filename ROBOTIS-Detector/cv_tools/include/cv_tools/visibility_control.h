#ifndef CV_TOOLS__VISIBILITY_CONTROL_H_
#define CV_TOOLS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CV_TOOLS_EXPORT __attribute__ ((dllexport))
    #define CV_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define CV_TOOLS_EXPORT __declspec(dllexport)
    #define CV_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CV_TOOLS_BUILDING_LIBRARY
    #define CV_TOOLS_PUBLIC CV_TOOLS_EXPORT
  #else
    #define CV_TOOLS_PUBLIC CV_TOOLS_IMPORT
  #endif
  #define CV_TOOLS_PUBLIC_TYPE CV_TOOLS_PUBLIC
  #define CV_TOOLS_LOCAL
#else
  #define CV_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define CV_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define CV_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define CV_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CV_TOOLS_PUBLIC
    #define CV_TOOLS_LOCAL
  #endif
  #define CV_TOOLS_PUBLIC_TYPE
#endif

#endif  // CV_TOOLS__VISIBILITY_CONTROL_H_
