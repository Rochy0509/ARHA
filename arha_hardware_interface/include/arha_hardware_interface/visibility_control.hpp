#ifndef ARHA_HARDWARE_VISIBILITY_CONTROL
#define ARHA_HARDWARE_VISIBILITY_CONTROL

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARHA_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define ARHA_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define ARHA_HARDWARE_EXPORT __declspec(dllexport)
    #define ARHA_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARHA_HARDWARE_BUILDING_LIBRARY
    #define ARHA_HARDWARE_PUBLIC ARHA_HARDWARE_EXPORT
  #else
    #define ARHA_HARDWARE_PUBLIC ARHA_HARDWARE_IMPORT
  #endif
  #define ARHA_HARDWARE_PUBLIC_TYPE ARHA_HARDWARE_PUBLIC
  #define ARHA_HARDWARE_LOCAL
#else
  #define ARHA_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define ARHA_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define ARHA_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define ARHA_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARHA_HARDWARE_PUBLIC
    #define ARHA_HARDWARE_LOCAL
  #endif
  #define ARHA_HARDWARE_PUBLIC_TYPE
#endif

#endif  // ARHA_HARDWARE_VISIBILITY_CONTROL