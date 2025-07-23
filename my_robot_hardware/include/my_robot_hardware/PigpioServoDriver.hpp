#ifndef PIGPIO_SERVO_DRIVER_HPP
#define PIGPIO_SERVO_DRIVER_HPP

#include <pigpiod_if2.h>
#include <iostream>
#include "my_robot_hardware/Result.hpp"
#include "my_robot_hardware/DriverError.hpp"


class PigpioServoDriver
{
public:

    /**
     * @param servo_pin Which pin is used for this servo?
     * @param min_pulse_width Defaults to 500 µs, which is the lowest it can be.
     * @param max_pulse_width Defaults to 2500 µs, which is the highest it can be.
     * @param min_angle Defaults to 0.0.
     * @param max_angle Defaults to 180.0.
     * @param host pigpiod daemon host. Defaults to localhost.
     * @param port pigpiod daemon port. Defaults to 8888.
     */
    PigpioServoDriver(
        const int &servo_pin,
        const int &min_pulse_width = PI_MIN_SERVO_PULSEWIDTH,
        const int &max_pulse_width = PI_MAX_SERVO_PULSEWIDTH,
        const double &min_angle = 0.0f,
        const double &max_angle = 180.0f,
        const std::string &host = "localhost",
        const std::string &port = "8888"
    )
    {
        servo_pin_ = servo_pin;
        min_pulse_width_ = min_pulse_width;
        max_pulse_width_ = max_pulse_width;
        min_angle_ = min_angle;
        max_angle_ = max_angle;
        host_ = host;
        port_ = port;
        pigpio_ = -1;
    }

    Result<bool, DriverError> connect()
    {
        pigpio_ = pigpio_start(host_.c_str(), port_.c_str());
        if (pigpio_ < 0)
        {
            return Result<bool, DriverError>::Err(DriverError::PigpioUnavailable);
        }

        set_mode(pigpio_, servo_pin_, PI_OUTPUT);
        return Result<bool, DriverError>::Ok(true);
    }

    // Disconnect from the pigpio daemon
    void disconnect()
    {
        if (pigpio_ >= 0)
        {
            // Stop servo pulses
            set_servo_pulsewidth(pigpio_, servo_pin_, 0);
            
            // Disconnect from daemon
            pigpio_stop(pigpio_);
            pigpio_ = -1;
        }
    }

    // Get the current pulse width from pigpio daemon
    int get_current_pulse_width() const
    {
        if (!is_connected()) return -1;
        return get_servo_pulsewidth(pigpio_, servo_pin_);
    }

    // Get the current angle (based on pulse width)
    float get_current_angle() const
    {
        int pulse = get_current_pulse_width();
        if (pulse < min_pulse_width_ || pulse > max_pulse_width_) return -1;
        // Linear mapping from pulse width to angle
        float angle = (float)(pulse - min_pulse_width_) / (max_pulse_width_ - min_pulse_width_) * 180.0f;
        return angle;
    }

    // Get the current normalized value [-1.0, 1.0]
    float get_current_value() const
    {
        int pulse = get_current_pulse_width();
        if (pulse < min_pulse_width_ || pulse > max_pulse_width_) return 0.0f;
        // Map pulse width to [-1.0, 1.0]
        float value = 2.0f * (float)(pulse - min_pulse_width_) / (max_pulse_width_ - min_pulse_width_) - 1.0f;
        return value;
    }

    void set_servo_angle(int pin, int angle)
    {
        // Clamp angle between 0 and 180
        if (angle < 0)
            angle = 0;
        if (angle > 180)
            angle = 180;
        // Map angle to pulse width (1000-2000us)
        int pulse_width = 1000 + (angle * 1000) / 180;
        set_servo_pulsewidth(pigpio_, pin, pulse_width);
    }

    // Move servo to center position
    void go_to_center(int pin)
    {
        int center_angle = (min_angle_ + max_angle_) / 2;
        go_to_angle(pin, center_angle);
    }

    // Move servo to lowest position
    void go_to_low(int pin)
    {
        go_to_angle(pin, min_angle_);
    }

    // Move servo to highest position
    void go_to_high(int pin)
    {
        go_to_angle(pin, max_angle_);
    }

    // Move servo to a normalized value [-1.0, 1.0]
    void go_to_value(int pin, float value)
    {
        if (value < -1.0f) value = -1.0f;
        if (value > 1.0f) value = 1.0f;
        float angle = min_angle_ + ((value + 1.0f) / 2.0f) * (max_angle_ - min_angle_);
        go_to_angle(pin, angle);
    }

    // Move servo to a specific angle
    void go_to_angle(int pin, float angle)
    {
        if (angle < min_angle_) angle = min_angle_;
        if (angle > max_angle_) angle = max_angle_;
        int pulse_width = min_pulse_width_ + ((angle - min_angle_) / (max_angle_ - min_angle_)) * (max_pulse_width_ - min_pulse_width_);
        set_servo_pulsewidth(pigpio_, pin, pulse_width);
    }

    // Check if connected to daemon
    bool is_connected() const
    {
        return pigpio_ >= 0;
    }

    // Destructor
    ~PigpioServoDriver()
    {
        disconnect();
    }

private:
    int servo_pin_;
    int min_pulse_width_;
    int max_pulse_width_;
    float min_angle_;
    float max_angle_;
    std::string host_;
    std::string port_;
    int pigpio_;
};

#endif