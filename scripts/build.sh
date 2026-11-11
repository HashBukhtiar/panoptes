#!/bin/bash

# Usage: `./scripts/build.sh [package_name]`

source /opt/ros/lyrical/setup.bash

if [ -z "$1" ]; then
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
else
    colcon build --packages-up-to "$1" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

echo "Build complete. If you added a new package, you must run 'source scripts/setup.sh' after to re-source your workspace"