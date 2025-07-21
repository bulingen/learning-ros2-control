#include "custom_robot_hardware/propeller_hardware_interface.hpp"
#include "custom_robot_hardware/DriverErrorToString.hpp"
#include "rclcpp/rclcpp.hpp"

#define PI 3.14159265359


namespace propeller_hardware
{
    hardware_interface::CallbackReturn PropellerHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(get_logger(), "Initiating... please wait...");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;

        pigpiod_host_ = info_.hardware_parameters["pigpiod_host"];
        pigpiod_port_ = info_.hardware_parameters["pigpiod_port"];

        driver_ = std::make_shared<PigpioDCMotorDriver>();

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn PropellerHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Configuring... please wait...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropellerHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {

        // TODO: maybe use actuator interface instead, since there is no feedback?

        // TODO: possibly move the connect to on_configure?
        // Reason to keep it here: I don't know what corresponding "down" callback there is.
        // on_activate has corresponding on_deactivate, where we wanna disconnect.
        // But what is the corresponding callback for on_configure?
        // TODO: the corresponding callback is on_cleanup. So use that.
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Activating... please wait...");

        auto connect_result = driver_->connect(pigpiod_host_, pigpiod_port_);
        if (!connect_result.is_ok())
        {
            std::string error_message = to_string(connect_result.error_code());
            RCLCPP_ERROR(get_logger(), "Connection failed to %s:%s: %s",
               pigpiod_host_.c_str(), pigpiod_port_.c_str(),
               error_message.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::cout << "Driver connected successfully." << std::endl;

        // Set states initially
        vel_ = 0.0;
        pos_ = 0.0;
        set_state("propeller_joint/velocity", vel_);
        set_state("propeller_joint/position", pos_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropellerHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "Deactivating... please wait...");
        driver_->disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    PropellerHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;

        vel_ = cmd_;
        pos_ += vel_ * period.seconds();

        vel_ = std::isnan(vel_) ? 0.0 : vel_;
        
        pos_ = std::isnan(pos_) ? 0.0 : pos_;

        set_state("propeller_joint/velocity", vel_);
        set_state("propeller_joint/position", pos_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    PropellerHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        double vel_cmd = get_command("propeller_joint/velocity");

        // RCLCPP_INFO(get_logger(), "COMMAND: %f", vel_cmd);

        cmd_ = vel_cmd;

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
        double requested_ratio_for_left_motor = vel_cmd / max_rad_per_sec;


        // RCLCPP_INFO(get_logger(), "WRITE. get command. right = %.2f left = %.2f", left_vel_cmd, right_vel_cmd);

        // NOTE: we're only controlling one motor here, since I only have one.
        // If we had a right wheel motor, we might've had to multiply that by -1
        // since the orientation of the motors are mirrored. Just a note for the "future".

        // RCLCPP_INFO(get_logger(), "RUN MOTOR: %f", requested_ratio_for_left_motor);

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
} // namespace propeller_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(propeller_hardware::PropellerHardwareInterface, hardware_interface::SystemInterface)
