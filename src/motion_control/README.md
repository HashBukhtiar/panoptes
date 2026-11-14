# motion_control

ROS 2 node responsible for converting camera orientation commands into joint commands. It subscribes to `camera_orientation_command` and `joint_state`, uses inverse kinematics to determine target joint angles, and generates rate-limited joint commands to move toward those targets.

## 1. Build and Run

### Build

```bash
colcon build --packages-select motion_control
source install/setup.bash
```

Alternatively, use the project scripts:

```bash
source scripts/setup.sh
./scripts/build.sh
```

### Run

```bash
ros2 run motion_control motion_control_node
```

Parameters can optionally be specified:

```bash
ros2 run motion_control motion_control_node --ros-args \
    -p loop_rate_hz:=100.0 \
    -p max_speed_rad_s:=0.5
```


## 2. ROS Interface

The node communicates through three ROS 2 topics.

| Topic | Direction | Message type | Purpose |
|---|---|---|---|
| `camera_orientation_command` | Subscribe | `interfaces/msg/CameraOrientationCommand` | Receives desired camera orientation. |
| `joint_state` | Subscribe | `sensor_msgs/msg/JointState` | Receives current joint positions for initialization. |
| `joint_command` | Publish | `interfaces/msg/JointCommand` | Publishes rate-limited joint positions. |

All three topics currently use best-effort reliability with a queue depth of 1. This prioritizes receiving the latest command or state rather than processing outdated messages.

Orientation commands follow the ROS REP-103 right-handed coordinate convention:

- X: Forward
- Y: Left
- Z: Up

Rotations follow the right-hand rule:

- Yaw: Rotation about +Z (CCW = positive).
- Pitch: Rotation about +Y (CCW = positive).
- Roll: Rotation about +X (CCW = positive).

All angular values must be supplied in radians.

## 3. Processing Pipeline

The node follows this sequence:

1. **Receive orientation:** Subscribe to `camera_orientation_command`.
2. **Calculate joint targets:** Pass the requested `(yaw, pitch, roll)` to `kinematics::inverse_kinematics()`.
3. **Store target:** If inverse kinematics succeeds, store the resulting joint angles and the command's timestamp as the new target.
4. **Initialize trajectory:** On the first received `joint_state`, initialize the commanded joint positions using the measured joint positions. This ensures that trajectory generation starts from the robot's current position.
5. **Generate trajectory:** At each update, move the commanded joint positions toward the target, subject to the maximum velocity constraint.
6. **Publish:** Publish the updated joint positions to `joint_command`.

## 4. Trajectory Generation and Velocity Limiting

The node uses a rate-limited trajectory generator to move commanded joint positions toward their targets without exceeding the configured maximum velocity.

### 4.1 Maximum Allowed Step

Let:
- `q_i`: Current commanded position of joint `i`.
- `q_i*`: Target position of joint `i`.
- `v_max`: Maximum allowed joint speed (rad/s).
- `dt`: Time elapsed since the previous trajectory update (s).

The maximum allowed movement per joint per update is:

    dq_max = v_max * dt

### 4.2 Position Update

First, calculate the largest position error across all joints:

    e_max = max_i |q_i* - q_i|

Then calculate a shared scaling factor:

    alpha = min(1, dq_max / e_max)

If all joints have reached their targets (`e_max = 0`), no update is needed.

The next commanded positions are:

    q_i_next = q_i + alpha * (q_i* - q_i)

This makes sure that no joint exceeds the maximum allowed movement while keeping synchronized progress toward the targets.

For example, given current positions `[0, 0]`, target positions `[1.0, 0.5]`, and a maximum step of `0.25 rad`:

    e_max = 1.0
    alpha = 0.25 / 1.0 = 0.25
    q_next = [0.25, 0.125]

This update is repeated until the target is reached.

**Note:** The current approach limits velocity but does not explicitly constrain acceleration or jerk.

## 5. Tests

GTest unit tests are located in `test/`.

Run the tests:

```bash
colcon test --packages-select motion_control
colcon test-result --verbose
```

Alternatively:

```bash
source scripts/setup.sh
./scripts/test.sh
```