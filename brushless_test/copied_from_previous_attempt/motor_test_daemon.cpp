#include <pigpiod_if2.h>
#include <iostream>
#include <unistd.h>

#define PWM_PIN 12
#define DIR_PIN 20
#define FORWARD_LEVEL 0

int main() {
    // Motor voltage logic (same as in motor_control.py)
    float input_voltage = 15.0;
    float motor_max_voltage = 12.0;
    int pwm_max = 255; // pigpio uses 0-255 for PWM
    float motor_ctl_level = (motor_max_voltage / input_voltage) * pwm_max;
    float signal = 1.0; // Full speed (can be adjusted 0.0 - 1.0)

    int pi = pigpio_start(NULL, NULL); // Connect to local pigpio daemon
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpio daemon" << std::endl;
        return 1;
    }
    std::cout << "Connected to pigpio daemon." << std::endl;

    set_mode(pi, DIR_PIN, PI_OUTPUT);
    set_mode(pi, PWM_PIN, PI_OUTPUT);

    // Set direction to forward
    gpio_write(pi, DIR_PIN, FORWARD_LEVEL);
    std::cout << "Direction set to FORWARD." << std::endl;

    // Start PWM
    set_PWM_dutycycle(pi, PWM_PIN, static_cast<int>(signal * motor_ctl_level));
    std::cout << "Motor running..." << std::endl;
    sleep(2); // Run for 2 seconds

    // Stop motor
    set_PWM_dutycycle(pi, PWM_PIN, 0);
    std::cout << "Motor stopped." << std::endl;

    pigpio_stop(pi); // Disconnect from daemon
    std::cout << "Disconnected from pigpio daemon." << std::endl;
    return 0;
} 