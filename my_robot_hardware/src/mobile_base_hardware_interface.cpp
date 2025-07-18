#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "my_robot_hardware/DriverErrorToString.hpp"

namespace mobile_base_hardware
{
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;

        driver_ = std::make_shared<PigpioDCMotorDriver>();

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        auto connect_result = driver_->connect();
        if (!connect_result.is_ok())
        {
            std::cerr << to_string(connect_result.error_code()) << std::endl;
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::cout << "Driver connected successfully." << std::endl;

        // Set states initially
        set_state("base_left_wheel_joint/velocity", 0.0);
        set_state("base_right_wheel_joint/velocity", 0.0);
        set_state("base_left_wheel_joint/position", 0.0);
        set_state("base_right_wheel_joint/position", 0.0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        driver_->disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    //     hardware_interface::return_type DiffBotSystemHardware::read(
    //   const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    // {
    //   // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    //   std::stringstream ss;
    //   ss << "Reading states:";
    //   for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    //   {
    //     // Simulate DiffBot wheels's movement as a first-order system
    //     // Update the joint status: this is a revolute joint without any limit.
    //     // Simply integrates
    //     hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    //     ss << std::fixed << std::setprecision(2) << std::endl
    //        << "\t"
    //           "position "
    //        << hw_positions_[i] << " and velocity " << hw_velocities_[i] << " for '"
    //        << info_.joints[i].name.c_str() << "'!";
    //   }
    //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    //   // END: This part here is for exemplary purposes - Please do not copy to your production code

    //   return hardware_interface::return_type::OK;
    // }

    hardware_interface::return_type
    MobileBaseHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;

        // TODO: need to store this in driver
        // double left_vel = 0.0;
        // double right_vel = 0.0;
        // double left_pos = 0.0;
        // double right_pos = 0.0;

        // hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
        // left_pos_ =

        // base_right_wheel_joint
        // base_left_wheel_joint
        set_state("base_left_wheel_joint/velocity", left_vel);
        set_state("base_right_wheel_joint/velocity", right_vel);
        set_state("base_left_wheel_joint/position", get_state("base_left_wheel_joint/position") + left_vel * period.seconds());
        set_state("base_right_wheel_joint/position", get_state("base_right_wheel_joint/position") + right_vel * period.seconds());

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    MobileBaseHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        driver_->forward(get_command("base_left_wheel_joint/velocity"));

        // TODO: need to handle right wheel somehow
        // TODO: also need to translate rad/s to speed ratio
        // driver_->forward(get_command("base_left_wheel_joint/velocity"))
        return hardware_interface::return_type::OK;
    }
} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
