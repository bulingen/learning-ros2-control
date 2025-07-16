#ifndef MOTOR_COMMS_HPP
#define MOTOR_COMMS_HPP

#include <pigpiod_if2.h>
#include <iostream>

#define PWM_PIN 12
#define DIR_PIN 20
#define FORWARD_LEVEL 0
#define BACKWARD_LEVEL 1

class MotorComms {
public:
    MotorComms()
        : pi_(-1), input_voltage_(15.0), motor_max_voltage_(12.0), pwm_max_(255) {
        motor_ctl_level_ = (motor_max_voltage_ / input_voltage_) * pwm_max_;
    }

    // Connect to the pigpio daemon
    bool connect() {
        pi_ = pigpio_start(NULL, NULL);
        if (pi_ < 0) {
            std::cerr << "Failed to connect to pigpio daemon" << std::endl;
            return false;
        }
        set_mode(pi_, DIR_PIN, PI_OUTPUT);
        set_mode(pi_, PWM_PIN, PI_OUTPUT);
        return true;
    }

    // Disconnect from the pigpio daemon
    void disconnect() {
        if (pi_ >= 0) {
            stop();
            pigpio_stop(pi_);
            pi_ = -1;
        }
    }

    // Move forward with a value between 0 and 1
    void forward(float signal) {
        if (!is_connected()) return;
        if (signal < 0) signal = 0;
        if (signal > 1) signal = 1;
        gpio_write(pi_, DIR_PIN, FORWARD_LEVEL);
        set_PWM_dutycycle(pi_, PWM_PIN, static_cast<int>(signal * motor_ctl_level_));
    }

    // Move backward with a value between 0 and 1
    void backward(float signal) {
        if (!is_connected()) return;
        if (signal < 0) signal = 0;
        if (signal > 1) signal = 1;
        gpio_write(pi_, DIR_PIN, BACKWARD_LEVEL);
        set_PWM_dutycycle(pi_, PWM_PIN, static_cast<int>(signal * motor_ctl_level_));
    }

    // Stop the motor
    void stop() {
        if (!is_connected()) return;
        set_PWM_dutycycle(pi_, PWM_PIN, 0);
    }

    // Check if connected to daemon
    bool is_connected() const {
        return pi_ >= 0;
    }

    ~MotorComms() {
        disconnect();
    }

private:
    int pi_;
    float input_voltage_;
    float motor_max_voltage_;
    int pwm_max_;
    float motor_ctl_level_;
};

#endif // MOTOR_COMMS_HPP 