#ifndef SOKI_HARDWARE__VISIBILITY_CONTROL_H_
#define SOKI_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SOKI_HARDWARE_EXPORT __attribute__((dllexport))
    #define SOKI_HARDWARE_IMPORT __attribute__((dllimport))
  #else
    #define SOKI_HARDWARE_EXPORT __declspec(dllexport)
    #define SOKI_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef SOKI_HARDWARE_BUILDING_DLL
    #define SOKI_HARDWARE_PUBLIC SOKI_HARDWARE_EXPORT
  #else
    #define SOKI_HARDWARE_PUBLIC SOKI_HARDWARE_IMPORT
  #endif
  #define SOKI_HARDWARE_LOCAL
#else
  #define SOKI_HARDWARE_EXPORT __attribute__((visibility("default")))
  #define SOKI_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define SOKI_HARDWARE_PUBLIC __attribute__((visibility("default")))
    #define SOKI_HARDWARE_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define SOKI_HARDWARE_PUBLIC
    #define SOKI_HARDWARE_LOCAL
  #endif
#endif

#endif  // SOKI_HARDWARE__VISIBILITY_CONTROL_H_
