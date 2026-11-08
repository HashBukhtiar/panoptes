#!/bin/bash

# Usage: `./scripts/build.sh [package_name]`

source /opt/ros/lyrical/setup.bash

if [ -z "$1" ]; then
    colcon build
else
    colcon build --packages-up-to "$1"
fi

echo "Build complete. To source the workspace, run: source scripts/setup.sh"