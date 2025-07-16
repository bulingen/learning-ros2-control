// #include "rclcpp/rclcpp.hpp"
#include "brushless_test/brushless_driver.hpp"
#include <thread>

using namespace std::chrono_literals;

int main()
{

    auto driver = BrushlessDriver();

    driver.connect();
    std::this_thread::sleep_for(1s);

    driver.forward(0.5);
    std::this_thread::sleep_for(3s);
    driver.stop();
    std::this_thread::sleep_for(1s);
    driver.backward(0.5);
    std::this_thread::sleep_for(3s);

    driver.disconnect();

    return 0;
}