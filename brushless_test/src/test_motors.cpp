// #include "rclcpp/rclcpp.hpp"
#include <thread>
#include <algorithm>
#include "brushless_test/brushless_driver.hpp"
#include "brushless_test/DriverErrorToString.hpp"

using namespace std::chrono_literals;

// # Convert command in rad/s to normalized speed [-1.0, 1.0]
// ratio = self.cmd_velocity / self.max_rad_per_sec
// ratio = max(min(ratio, 1.0), -1.0)
// self.motor.set_duty_cycle(ratio)

#define PI 3.14159265359

int main()
{
    // TODO: move this to driver?
    int max_rpm = 350;
    float max_revolutions_per_second = max_rpm / 60.0;
    float rad_per_revolution = 2 * PI;
    float max_rad_per_sec = max_revolutions_per_second * rad_per_revolution;

    std::cout << "max_rad_per_sec" << max_rad_per_sec << std::endl;

    // this is mocked command we get
    float one_lap = 2 * PI;
    float cmd_vel_rad_per_sec = 10 * one_lap;

    std::cout << "cmd_vel_rad_per_sec" << cmd_vel_rad_per_sec << std::endl;

    // handle command
    double requested_ratio = cmd_vel_rad_per_sec / max_rad_per_sec;
    std::cout << "requested_ratio" << requested_ratio << std::endl;
    float capped_ratio = std::max(std::min(requested_ratio, 1.0), -1.0);
    std::cout << "capped_ratio" << capped_ratio << std::endl;

    // TODO: probably need to do conversion of rad/s to speed of motor here
    // knowing that the motor runs at 350 rpm

    auto driver = BrushlessDriver();

    auto result = driver.connect();

    if (!result.is_ok())
    {
        std::cerr << to_string(result.error_code()) << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Driver connected successfully." << std::endl;
    }

    std::this_thread::sleep_for(1s);

    driver.forward(capped_ratio);
    std::this_thread::sleep_for(3s);
    // driver.stop();
    // std::this_thread::sleep_for(1s);
    // driver.backward(0.5);
    // std::this_thread::sleep_for(3s);

    driver.disconnect();

    return 0;
}