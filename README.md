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

ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

```

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
