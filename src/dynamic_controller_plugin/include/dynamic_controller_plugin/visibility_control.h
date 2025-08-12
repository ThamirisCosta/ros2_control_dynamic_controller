#ifndef DYNAMIC_CONTROLLER_PLUGIN__VISIBILITY_CONTROL_H_
#define DYNAMIC_CONTROLLER_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COMPUTED_TORQUE_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define COMPUTED_TORQUE_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define COMPUTED_TORQUE_CONTROLLER_EXPORT __declspec(dllexport)
    #define COMPUTED_TORQUE_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_CONTROLLER_PLUGIN_BUILDING_LIBRARY
    #define COMPUTED_TORQUE_CONTROLLER_PUBLIC COMPUTED_TORQUE_CONTROLLER_EXPORT
  #else
    #define COMPUTED_TORQUE_CONTROLLER_PUBLIC COMPUTED_TORQUE_CONTROLLER_IMPORT
  #endif
  #define COMPUTED_TORQUE_CONTROLLER_PUBLIC_TYPE COMPUTED_TORQUE_CONTROLLER_PUBLIC
  #define COMPUTED_TORQUE_CONTROLLER_LOCAL
#else
  #define COMPUTED_TORQUE_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define COMPUTED_TORQUE_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define COMPUTED_TORQUE_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define COMPUTED_TORQUE_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COMPUTED_TORQUE_CONTROLLER_PUBLIC
    #define COMPUTED_TORQUE_CONTROLLER_LOCAL
  #endif
  #define COMPUTED_TORQUE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // DYNAMIC_CONTROLLER_PLUGIN__VISIBILITY_CONTROL_H_
