# interfaces

Message types for the pub/sub topics. Custom messages live in `msg/`. Standard ROS 2 types are used wherever one already exists.

## Topics and types

| Topic | Type | Flow |
|---|---|---|
| camera orientation command | `interfaces/msg/CameraOrientationCommand` (custom) | teleop-input, auto-detection-tracking -> motion-control |
| joint command | `interfaces/msg/JointCommand` (custom) | motion-control -> actuator-driver |
| joint state | `sensor_msgs/msg/JointState` (standard) | actuator-driver -> motion-control |
| current camera image | `sensor_msgs/msg/Image` (standard) | camera-driver -> auto-detection-tracking, teleop-display |

## Conventions

- **Units:** radians
- **Zero pose:** camera facing forward and level relative to the torso
- **Timestamps:** commands are stamped when created; state is stamped when measured

## Look up a type

```
ros2 interface show <TYPE>
```
