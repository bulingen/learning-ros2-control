#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/PigpioDCMotorDriver.hpp"

namespace mobile_base_hardware
{

    class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // Lifecycle node override
        hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // SystemInterface override
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo &info) override;

        hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::shared_ptr<PigpioDCMotorDriver> driver_;
    }; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware

#endif