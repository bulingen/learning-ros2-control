#pragma once
#include "custom_robot_hardware/DriverError.hpp"
#include <string>

inline std::string to_string(DriverError err)
{
    switch (err)
    {
    case DriverError::None:
        return "No error";
    case DriverError::PigpioUnavailable:
        return "Pigpio unavailable";
    case DriverError::Unknown:
        return "Unknown error";
    default:
        return "Unrecognized error";
    }
}