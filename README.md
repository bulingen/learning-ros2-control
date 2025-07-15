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
