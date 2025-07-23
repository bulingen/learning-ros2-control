#include "my_robot_hardware/arm_hardware_interface.hpp"
#include "my_robot_hardware/DriverErrorToString.hpp"
#include <cmath>

namespace arm_hardware
{
    hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(get_logger(), "Initiating... please wait...");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;

        servo_pin_1_ = info_.hardware_parameters.count("servo_pin_1")
                           ? std::stoi(info_.hardware_parameters["servo_pin_1"])
                           : 13;

        servo_pin_2_ = info_.hardware_parameters.count("servo_pin_2")
                           ? std::stoi(info_.hardware_parameters["servo_pin_2"])
                           : 19;

        min_pulse_width_ = info_.hardware_parameters.count("min_pulse_width")
                               ? std::stoi(info_.hardware_parameters["min_pulse_width"])
                               : 800;

        max_pulse_width_ = info_.hardware_parameters.count("max_pulse_width")
                               ? std::stoi(info_.hardware_parameters["max_pulse_width"])
                               : 2200;

        min_angle_ = info_.hardware_parameters.count("min_angle")
                         ? hardware_interface::stod(info_.hardware_parameters["min_angle"])
                         : 0.0f;

        max_angle_ = info_.hardware_parameters.count("max_angle")
                         ? hardware_interface::stod(info_.hardware_parameters["max_angle"])
                         : 140.0f;

        pigpiod_host_ = info_.hardware_parameters.count("pigpiod_host")
                            ? info_.hardware_parameters["pigpiod_host"]
                            : "localhost";

        pigpiod_port_ = info_.hardware_parameters.count("pigpiod_port")
                            ? info_.hardware_parameters["pigpiod_port"]
                            : "8888";

        driver_1_ = std::make_shared<PigpioServoDriver>(
            servo_pin_1_,
            min_pulse_width_,
            max_pulse_width_,
            min_angle_,
            max_angle_,
            pigpiod_host_,
            pigpiod_port_);
        driver_2_ = std::make_shared<PigpioServoDriver>(
            servo_pin_2_,
            min_pulse_width_,
            max_pulse_width_,
            min_angle_,
            max_angle_,
            pigpiod_host_,
            pigpiod_port_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Configuring... please wait...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Activating... please wait...");

        auto connect_result_1 = driver_1_->connect();
        auto connect_result_2 = driver_2_->connect();
        if (!connect_result_2.is_ok() || !connect_result_2.is_ok())
        {
            std::string error_message = to_string(connect_result_1.error_code());
            RCLCPP_ERROR(get_logger(), "Connection failed to %s:%s: %s",
                         pigpiod_host_.c_str(), pigpiod_port_.c_str(),
                         error_message.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::cout << "Driver connected successfully." << std::endl;

        // Set states initially
        // left_vel_ = 0.0;
        // left_pos_ = 0.0;
        // right_vel_ = 0.0;
        // right_pos_ = 0.0;
        // set_state("base_left_wheel_joint/velocity", left_vel_);
        // set_state("base_left_wheel_joint/position", left_pos_);
        // set_state("base_right_wheel_joint/velocity", right_vel_);
        // set_state("base_right_wheel_joint/position", right_pos_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Deactivating... please wait...");
        driver_1_->disconnect();
        driver_2_->disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    ArmHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        auto pos_1_deg = driver_1_->get_current_angle();
        auto pos_2_deg = driver_2_->get_current_angle();

        double pos_1_rad = pos_1_deg * M_PI / 180.0;
        double pos_2_rad = pos_2_deg * M_PI / 180.0;

        // RCLCPP_INFO(get_logger(), "READ - set state pos1: %f", pos_1_rad);

        set_state("arm_joint1/position", pos_1_rad);
        set_state("arm_joint2/position", pos_2_rad);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    ArmHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        double pos_1_cmd = get_command("arm_joint1/position");
        double pos_2_cmd = get_command("arm_joint2/position");

        // RCLCPP_INFO(get_logger(), "WRITE - command pos1: %f", pos_1_cmd);

        // double pos1rad = pos_1_cmd * M_PI / 180.0;
        // double pos2rad = pos_2_cmd * M_PI / 180.0;
        double degrees1 = pos_1_cmd * 180.0 / M_PI;
        double degrees2 = pos_2_cmd * 180.0 / M_PI;

        driver_1_->set_servo_angle(degrees1);
        driver_2_->set_servo_angle(degrees2);

        return hardware_interface::return_type::OK;
    }

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
