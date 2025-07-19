#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"

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

        // Export interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:
        std::shared_ptr<PigpioDCMotorDriver> driver_;

        // // Parameters for the DiffBot simulation
        // double hw_start_sec_;
        // double hw_stop_sec_;

        // // Objects for logging
        // std::shared_ptr<rclcpp::Logger> logger_;
        // rclcpp::Clock::SharedPtr clock_;

        // // Store the command for the simulated robot
        // std::vector<double> hw_commands_;
        // std::vector<double> hw_positions_;
        // std::vector<double> hw_velocities_;

        // States
        double left_vel_;
        double right_vel_;

        double left_pos_;
        double right_pos_;

        std::shared_ptr<rclcpp::Logger> logger_;
        rclcpp::Clock::SharedPtr clock_;

        // Commands
        double left_vel_cmd_;
        double right_vel_cmd_;

        double left_pos_cmd_;
        double right_pos_cmd_;

    }; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware

#endif