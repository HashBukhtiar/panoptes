#!/bin/bash

# Usage: `./scripts/test.sh [package_name]`
source /opt/ros/lyrical/setup.bash

if [ -z "$1" ]; then
    colcon test --ctest-args -L 'gtest|pytest'
else
    colcon test --packages-select "$1" --ctest-args -L 'gtest|pytest'
fi

echo ""
echo "========== Test Results =========="
colcon test-result --all