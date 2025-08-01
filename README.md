# Get started

```bash
# create a workspace
cd path/to/where_ever
mkdir course_ws

# clone this repo
cd course_ws
git clone https://github.com/bulingen/learning-ros2-control.git

# rename folder to src
mv learning-ros2-control src

# source ros
source /opt/ros/jazzy/setup.zsh

# go back to ws and build
colcon build
```

## Run things

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /absolute/path/to/ws/src/my_robot_description/urdf/custom_robot.urdf.xacro)"

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

Or, the easiest way:

```bash
ros2 launch custom_robot_bringup custom_robot.launch.xml
ros2 run custom_robot_teleop cmd_vel_to_single_array
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p rate:=20
```

## Display custom robot

```bash
ros2 launch custom_robot_description display.launch.xml
```

## Launch custom robot in gazebo

```bash
ros2 launch custom_robot_bringup custom_robot.gazebo.launch.xml
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

## VS Code tips

Run this:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

And then add to your `c_cpp_properties.json`:

```diff
{
    "configurations": [
        {
            "browse": {
                "limitSymbolsToIncludedHeaders": true,
                "path": [
                    "${workspaceFolder}/**"
                ]
            },
            "name": "ros2",
            "intelliSenseMode": "gcc-x64",
            "cStandard": "gnu11",
            "cppStandard": "c++17",
+            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
    ],
    "version": 4
}
```

## Course project: controlling arm

**Run like this**

```bash
ros2 launch my_robot_bringup my_robot.launch.xml

ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 1.5]"
```

Note that I get some error when spawning arm controller in launch file. If I comment that out and spawn it manually, at least it starts up.
But cannot get the arm to move though :(

Okay, it's actually working now. When posting [1.57, 1.57], both joints go to 90 degrees, correctly.
However, when posting [0.0, 0.0], it doesn't go straight. See screenshot in images folder.

But otherwise, it's working. Run launch file like above (assuming arm controller isn't spawned), then spawn arm controller.
Then run rviz and the pub command above. Fun times.

## TODO

- Check here, for CI: https://github.com/botamochi6277/ros2_pigpio/blob/main/.github/workflows/main.yml
- Shift to using radians instead of degrees, for servo drivers and interfaces and everything really

## Notes regarding servos

- I measured the pulse widths and angles of the 577 and 507 servos:
    - 500 - 2500 is making a noise at the boundaries. So that suggests that it's too far.
    - 800 - 2200 works (and is close to our previous values of 810 - 2200).
    - when using 800 - 2200 I got these angles:
        - (measuring with a protractor) assuming center is at 90 degrees
        - then floor is at 20 degrees
        - and ceiling is at 160 degrees
        - that gives us rotational angle of 140 degrees
        - with 70 degrees from floor to midpoint, and from midpoint to ceiling.
        - so we could rather say:
            - center at 0 degrees
            - then -70 deg anti-clockwise
            - and +70 deg clockwise


## Notes on running with Gazebo

First, install stuff:

```bash
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control
```

Then launch:

```bash
ros2 launch my_robot_bringup my_robot.gazebo.launch.xml
```

And drive with:

```bash
ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true -p rate:=20

ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]"
```
