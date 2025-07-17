#pragma once
#include "brushless_test/DriverError.hpp"
#include <string>

inline std::string to_string(DriverError err)
{
    switch (err)
    {
    case DriverError::None:
        return "No error";
    case DriverError::PigpioUnavailable:
        return "Failed to connect to pigpio daemon. Is it running?";
    case DriverError::Unknown:
        return "Unknown error";
    default:
        return "Unrecognized error";
    }
}