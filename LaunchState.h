#ifndef LaunchState_h
#include <Arduino.h>
enum class LaunchState{
    PreIgnition,
    Ignition,
    Apogee,
    steadyDescent,
    Thousand_ft,
    Descent,
    Touchdown,
};
#endif