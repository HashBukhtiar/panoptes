# motion_control

ROS 2 node that turns camera orientation commands into joint commands. It subscribes to `camera_orientation_command` and `joint_state`, converts the target orientation to joint angles with inverse kinematics, and generates the joint commands to reach them.

## Run the node

```
colcon build --packages-select motion_control
source install/setup.bash
ros2 run motion_control motion_control
```

OR

```
source scripts/setup.sh
./scripts/build.sh
ros2 run motion_control motion_control
```

You can optionally specify parameters `loop_rate_hz` (default 100.0) and `max_speed_rad_s` (default 0.5).

```
ros2 run motion_control motion_control --ros-args -p max_speed_rad_s:=0.3
```

## Run the tests

gtest tests live in `test/`.

```
colcon test --packages-select motion_control
colcon test-result --verbose
```

OR

```
source scripts/setup.sh
./scripts/test.sh
```