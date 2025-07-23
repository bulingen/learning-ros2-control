#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/PigpioServoDriver.hpp"

namespace arm_hardware
{

    class ArmHardwareInterface : public hardware_interface::SystemInterface
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
        std::shared_ptr<PigpioServoDriver> driver_1_;
        std::shared_ptr<PigpioServoDriver> driver_2_;

        int servo_pin_1_;
        int servo_pin_2_;
        int min_pulse_width_;
        int max_pulse_width_;
        double min_angle_;
        double max_angle_;
        std::string pigpiod_host_;
        std::string pigpiod_port_;

        // States
        double joint_1_pos_ = 0.0;
        double joint_2_pos_ = 0.0;

        // Commands
        double joint_1_cmd_ = 0.0;
        double joint_2_cmd_ = 0.0;

    }; // class ArmHardwareInterface

} // namespace arm_hardware

#endif