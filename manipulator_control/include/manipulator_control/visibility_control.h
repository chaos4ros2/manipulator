#ifndef MANIPULATOR_CONTROL__VISIBILITY_CONTROL_H_
#define MANIPULATOR_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MANIPULATOR_CONTROL_EXPORT __attribute__((dllexport))
#define MANIPULATOR_CONTROL_IMPORT __attribute__((dllimport))
#else
#define MANIPULATOR_CONTROL_EXPORT __declspec(dllexport)
#define MANIPULATOR_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef MANIPULATOR_CONTROL_BUILDING_DLL
#define MANIPULATOR_CONTROL_PUBLIC MANIPULATOR_CONTROL_EXPORT
#else
#define MANIPULATOR_CONTROL_PUBLIC MANIPULATOR_CONTROL_IMPORT
#endif
#define MANIPULATOR_CONTROL_PUBLIC_TYPE MANIPULATOR_CONTROL_PUBLIC
#define MANIPULATOR_CONTROL_LOCAL
#else
#define MANIPULATOR_CONTROL_EXPORT __attribute__((visibility("default")))
#define MANIPULATOR_CONTROL_IMPORT
#if __GNUC__ >= 4
#define MANIPULATOR_CONTROL_PUBLIC __attribute__((visibility("default")))
#define MANIPULATOR_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define MANIPULATOR_CONTROL_PUBLIC
#define MANIPULATOR_CONTROL_LOCAL
#endif
#define MANIPULATOR_CONTROL_PUBLIC_TYPE
#endif

#endif  // MANIPULATOR_CONTROL__VISIBILITY_CONTROL_H_