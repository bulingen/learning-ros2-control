#ifndef PROPELLER_HARDWARE_INTERFACE_HPP
#define PROPELLER_HARDWARE_INTERFACE_HPP

#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"

#include "hardware_interface/system_interface.hpp"
#include "custom_robot_hardware/PigpioDCMotorDriver.hpp"

namespace propeller_hardware
{

    class PropellerHardwareInterface : public hardware_interface::SystemInterface
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

        std::string pigpiod_host_ = "localhost";
        std::string pigpiod_port_ = "8888";

        // States
        double vel_ = 0.0;
        double pos_ = 0.0;

        // Commands
        double cmd_ = 0.0;
    }; // class PropellerHardwareInterface

} // namespace propeller_hardware

#endif