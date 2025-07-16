#include <pigpio.h>
#include <iostream>
#include <unistd.h> // for sleep

#define PWM_PIN 12
#define DIR_PIN 20

#define FORWARD_LEVEL 0
#define BACKWARD_LEVEL 1

int main() {
    // Motor voltage logic (same as in motor_control.py)
    float input_voltage = 15.0;
    float motor_max_voltage = 12.0;
    int pwm_max = 255; // pigpio uses 0-255 for PWM
    float motor_ctl_level = (motor_max_voltage / input_voltage) * pwm_max;
    float signal = 1.0; // Full speed (can be adjusted 0.0 - 1.0)

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed" << std::endl;
        return 1;
    }
    std::cout << "pigpio initialized successfully!" << std::endl;

    gpioSetMode(DIR_PIN, PI_OUTPUT);
    gpioSetMode(PWM_PIN, PI_OUTPUT);

    // Set direction to forward
    gpioWrite(DIR_PIN, FORWARD_LEVEL);
    std::cout << "Direction set to FORWARD." << std::endl;

    // Start PWM
    gpioPWM(PWM_PIN, static_cast<int>(signal * motor_ctl_level));
    std::cout << "Motor running..." << std::endl;
    sleep(2); // Run for 2 seconds

    // Stop motor
    gpioPWM(PWM_PIN, 0);
    std::cout << "Motor stopped." << std::endl;

    gpioTerminate();
    std::cout << "pigpio terminated." << std::endl;
    return 0;
} 