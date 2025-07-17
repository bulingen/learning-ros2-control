#ifndef BRUSHLESS_DRIVER_HPP
#define BRUSHLESS_DRIVER_HPP

#include <pigpiod_if2.h>
#include <iostream>
#include "brushless_test/Result.hpp"
#include "brushless_test/DriverError.hpp"

#define PWM_PIN 12
#define DIR_PIN 20
#define FORWARD_LEVEL 0
#define BACKWARD_LEVEL 1

class BrushlessDriver
{
public:
    BrushlessDriver()
    {
        // : pi_(-1), input_voltage_(15.0), motor_max_voltage_(12.0), pwm_max_(255) {
        // motor_ctl_level_ = (motor_max_voltage_ / input_voltage_) * pwm_max_;
        pigpio_ = -1;

        // this is the input voltage on alen
        // input_voltage_ = 15.0;

        // this is what I think is input on tvalen
        input_voltage_ = 12.0;

        motor_max_voltage_ = 12.0;
        pwm_max_ = 255;
        motor_ctl_level_ = (motor_max_voltage_ / input_voltage_) * pwm_max_;
        std::cout << "motor_ctl_level_" << motor_ctl_level_ << std::endl;
    }

    /**
     * Connect to running pigpiod daemon.
     *
     * @param host pigpiod daemon host. Defaults to localhost.
     * @param port pigpiod daemon port. Defaults to 8888.
     */
    Result<bool, DriverError> connect(const char *host = "localhost", const char *port = "8888")
    {
        pigpio_ = pigpio_start(host, port);

        if (pigpio_ < 0)
        {
            // std::cerr << "Failed to connect to pigpio daemon" << std::endl;
            // return false;
            return Result<bool, DriverError>::Err(DriverError::PigpioUnavailable);
        }
        set_mode(pigpio_, DIR_PIN, PI_OUTPUT);
        set_mode(pigpio_, PWM_PIN, PI_OUTPUT);
        // return true;
        return Result<bool, DriverError>::Ok(true);
    }

    // Disconnect from the pigpio daemon
    void disconnect()
    {
        if (pigpio_ >= 0)
        {
            stop();
            pigpio_stop(pigpio_);
            pigpio_ = -1;
        }
    }

    // Move forward with a value between 0 and 1
    void forward(float signal)
    {
        if (!is_connected())
            return;
        if (signal < 0)
            signal = 0;
        if (signal > 1)
            signal = 1;
        gpio_write(pigpio_, DIR_PIN, FORWARD_LEVEL);
        float dutycycle = signal * motor_ctl_level_;
        std::cout << "dutycycle" << dutycycle << std::endl;
        int dutycycleint = static_cast<int>(signal * motor_ctl_level_);
        std::cout << "dutycycleint" << dutycycleint << std::endl;
        set_PWM_dutycycle(pigpio_, PWM_PIN, dutycycleint);
        // set_PWM_dutycycle(pigpio_, PWM_PIN, 255);
    }

    // Move backward with a value between 0 and 1
    void backward(float signal)
    {
        if (!is_connected())
            return;
        if (signal < 0)
            signal = 0;
        if (signal > 1)
            signal = 1;
        gpio_write(pigpio_, DIR_PIN, BACKWARD_LEVEL);
        set_PWM_dutycycle(pigpio_, PWM_PIN, static_cast<int>(signal * motor_ctl_level_));
    }

    // void BrushlessMotor::forward(double target_rad_per_sec) {
    //     double clamped = std::clamp(target_rad_per_sec, -max_rads, max_rads);
    //     double pwm = (clamped / max_rads) * pwm_range;  // Map [-max_rads, max_rads] to [min_pwm, max_pwm]
    //     set_pwm(pwm);
    // }

    // def set_duty_cycle(self, duty_cycle: float):
    //     """Set the motor speed as a percentage [-1.0, 1.0]"""
    //     ...

    // def set_pwm(self, pwm_microseconds: int):
    //     """Directly set the PWM pulse width, e.g. 1500us"""

    // Stop the motor
    void stop()
    {
        if (!is_connected())
            return;
        set_PWM_dutycycle(pigpio_, PWM_PIN, 0);
    }

    // Check if connected to daemon
    bool is_connected() const
    {
        return pigpio_ >= 0;
    }

    // Destructor
    ~BrushlessDriver()
    {
        disconnect();
    }

private:
    int pigpio_;
    float input_voltage_;
    float motor_max_voltage_;
    int pwm_max_;
    float motor_ctl_level_;
};

#endif