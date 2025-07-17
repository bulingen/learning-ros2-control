// #include "rclcpp/rclcpp.hpp"
#include "brushless_test/brushless_driver.hpp"
#include <thread>
#include "brushless_test/DriverErrorToString.hpp"

using namespace std::chrono_literals;

int main()
{

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

    driver.forward(1.0);
    std::this_thread::sleep_for(3s);
    driver.stop();
    std::this_thread::sleep_for(1s);
    driver.backward(0.5);
    std::this_thread::sleep_for(3s);

    driver.disconnect();

    return 0;
}