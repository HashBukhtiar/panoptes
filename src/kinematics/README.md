# kinematics

Library for converting between camera orientation (yaw, pitch, roll) and joint angles.

- `inverse_kinematics(orientation)`: camera orientation to joint angles
- `forward_kinematics(joint_angles)`: joint angles to camera orientation

Both return `std::nullopt` when no solution exists.

## Include
In your program:

```cpp
#include "kinematics/kinematics.hpp"
```

In your package's `CMakeLists.txt`:

```cmake
find_package(kinematics REQUIRED)
target_link_libraries(<your_target> kinematics::kinematics)
```

In your `package.xml`:

```xml
<depend>kinematics</depend>
```