#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "my_robot_hardware/DriverErrorToString.hpp"
#include "rclcpp/rclcpp.hpp"

#define PI 3.14159265359


namespace mobile_base_hardware
{
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(get_logger(), "Initiating... please wait...");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;

        driver_ = std::make_shared<PigpioDCMotorDriver>();

        // left_vel_ = 0.0;
        // left_pos_ = 0.0;
        // right_vel_ = 0.0;
        // right_pos_ = 0.0;


        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Configuring... please wait...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {

        // TODO: maybe use actuator interface instead, since there is no feedback?

        // TODO: possibly move the connect to on_configure?
        // Reason to keep it here: I don't know what corresponding "down" callback there is.
        // on_activate has corresponding on_deactivate, where we wanna disconnect.
        // But what is the corresponding callback for on_configure?
        // TODO: the corresponding callback is on_cleanup. So use that.
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Activating... please wait...");

        auto connect_result = driver_->connect();
        if (!connect_result.is_ok())
        {
            std::cerr << to_string(connect_result.error_code()) << std::endl;
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::cout << "Driver connected successfully." << std::endl;

        // Set states initially
        left_vel_ = 0.0;
        left_pos_ = 0.0;
        right_vel_ = 0.0;
        right_pos_ = 0.0;
        set_state("base_left_wheel_joint/velocity", left_vel_);
        set_state("base_left_wheel_joint/position", left_pos_);
        set_state("base_right_wheel_joint/velocity", right_vel_);
        set_state("base_right_wheel_joint/position", right_pos_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Deactivating... please wait...");
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
        // set_state("base_left_wheel_joint/velocity", left_vel_);
        // set_state("base_right_wheel_joint/velocity", right_vel_);

        // double prev_left_vel = get_state("base_left_wheel_joint/velocity");
        // double prev_right_vel = get_state("base_right_wheel_joint/velocity");
        // double prev_left_pos = get_state("base_left_wheel_joint/position");
        // double prev_right_pos = get_state("base_right_wheel_joint/position");
        // NOTE: this is what made the thing work, although not that useful
        // set_state("base_left_wheel_joint/velocity", left_vel_);
        // set_state("base_right_wheel_joint/velocity", right_vel_);
        // set_state("base_left_wheel_joint/position", prev_left_pos + left_vel_ * period.seconds());
        // set_state("base_right_wheel_joint/position", prev_right_pos + right_vel_ * period.seconds());


        left_vel_ = left_vel_cmd_;
        right_vel_ = right_vel_cmd_;
        left_pos_ += left_vel_ * period.seconds();
        right_pos_ += right_vel_ * period.seconds();

        left_vel_ = std::isnan(left_vel_) ? 0.0 : left_vel_;
        right_vel_ = std::isnan(right_vel_) ? 0.0 : right_vel_;
        
        left_pos_ = std::isnan(left_pos_) ? 0.0 : left_pos_;
        right_pos_ = std::isnan(right_pos_) ? 0.0 : right_pos_;


        set_state("base_left_wheel_joint/velocity", left_vel_);
        set_state("base_left_wheel_joint/position", left_pos_);
        set_state("base_right_wheel_joint/velocity", right_vel_);
        set_state("base_right_wheel_joint/position", right_pos_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    MobileBaseHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        double left_vel_cmd = get_command("base_left_wheel_joint/velocity");
        double right_vel_cmd = get_command("base_right_wheel_joint/velocity");
        // (void)right_vel_cmd; // voiding this to avoid warnings for being unused

        left_vel_cmd_ = left_vel_cmd;
        right_vel_cmd_ = right_vel_cmd;

        // NOTE:
        // The command is coming in at rad/s. The diff controller has already calculated what it should be, based on
        // what the /cmd_vel topic gets (from teleopkeyboard for example). So if keyboard says "0.5 m/s",
        // then diff controller maybe sees that wheel radius is 0.1 m. The resulting rotational speed of the wheel
        // is then 5.0 rad/s.

        // What we need to do then, is to convert that to how fast the dc motor should spin.
        // It should spin 5 rad/s which is 5/2PI = 0.796 rotations per second
        // Our motor's max speed is 350 RPM.
        double max_rpm = 350.0;
        double max_revolutions_per_second = max_rpm / 60.0;
        double rad_per_revolution = 2 * PI;
        double max_rad_per_sec = max_revolutions_per_second * rad_per_revolution;


        // this is mocked command we get
        // float one_lap = 2 * PI;
        // float cmd_vel_rad_per_sec = 10 * one_lap;

        // handle command
        // the command is capped in the driver anyway
        double requested_ratio_for_left_motor = left_vel_cmd / max_rad_per_sec;


        // RCLCPP_INFO(get_logger(), "WRITE. get command. right = %.2f left = %.2f", left_vel_cmd, right_vel_cmd);

        // NOTE: we're only controlling one motor here, since I only have one.
        // If we had a right wheel motor, we might've had to multiply that by -1
        // since the orientation of the motors are mirrored. Just a note for the "future".
        driver_->run_motor(requested_ratio_for_left_motor);


        // if (left_vel_cmd > 0.0) {
        //     driver_->forward(left_vel_cmd);
        // } else if (left_vel_cmd < 0.0) {
        //     driver_->backward(left_vel_cmd);
        // }
        
        // left_vel_ = left_vel_cmd;

        // right_vel_ = right_vel_cmd;

        // TODO: need to handle right wheel somehow
        // TODO: also need to translate rad/s to speed ratio
        // driver_->forward(get_command("base_left_wheel_joint/velocity"))
        return hardware_interface::return_type::OK;
    }
} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
