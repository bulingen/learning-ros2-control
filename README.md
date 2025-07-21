# Get started

```bash
# create a workspace
cd path/to/where_ever
mkdir -p my_ws/src

# clone this repo
cd my_ws/src
git clone https://github.com/bulingen/learning-ros2-control.git

# source ros
source /opt/ros/humble/setup.zsh

# go back to ws and build
cd ../..
colcon build
```

## Run things

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /absolute/path/to/ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"

ros2 run controller_manager ros2_control_node --ros-args --remap /controller_manager/robot_description:=/robot_description --params-file /absolute/path/to/ws/src/my_robot_bringup/config/my_robot_controllers.yaml

ros2 run controller_manager spawner joint_state_broadcaster

ros2 run controller_manager spawner diff_drive_controller

ros2 run rviz2 rviz2 -d /absolute/path/to/ws/src/my_robot_description/rviz/urdf_config.rviz

ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true -p rate:=20

```

Notice the `-p rate:=20` param. Sometimes messages are dropped, or I get "jumpy" motor behaviour. With a rate of 20, those problems go away.

## Run with launch files

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```

## Other useful commands

```bash
ros2 control list_controllers # show active/inactive controllers at the moment
ros2 control list_controller_types # show available controllers
ros2 control list_hardware_interfaces # show running hardware interfaces
ros2 control list_hardware_components # show running components
```

## Make propeller move in custom robot

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /absolute/path/to/ws/src/custom_robot_description/urdf/custom_robot.urdf.xacro)"

ros2 run controller_manager ros2_control_node --ros-args --remap /controller_manager/robot_description:=/robot_description --params-file /absolute/path/to/ws/src/custom_robot_bringup/config/custom_robot_controllers.yaml

ros2 run controller_manager spawner joint_state_broadcaster

ros2 run controller_manager spawner velocity_controller

ros2 run rviz2 rviz2 -d /absolute/path/to/ws/src/custom_robot_description/rviz/config2.rviz

ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]"
```

You can also use `teleop_twist_keyboard` like this:

```bash
ros2 run custom_robot_teleop cmd_vel_to_single_array
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p rate:=20
```

## Install pigpio

Pigpio is not available in the default Ubuntu apt repository. One could install it from source, from here: https://abyz.me.uk/rpi/pigpio/download.html

But to make pigpio more discoverable from ROS and whatnot, one can instead install it from the raspberry-pi repository, like this:

```bash

curl -fsSL https://archive.raspberrypi.org/debian/raspberrypi.gpg.key | \
  gpg --dearmor | sudo tee /usr/share/keyrings/raspberrypi-archive-keyring.gpg > /dev/null

echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.org/debian bullseye main" | \
  sudo tee /etc/apt/sources.list.d/raspi.list > /dev/null

sudo apt update
```

Then:

```bash
sudo apt install pigpio libpigpio-dev
```

To start and stop the daemon:

```bash
sudo pigpiod
sudo killall pigpiod
```

Maybe your user should also be in the gpio group? Dunno.

## TODO

- Check here, for CI: https://github.com/botamochi6277/ros2_pigpio/blob/main/.github/workflows/main.yml
