#!/bin/bash

# Usage: `./scripts/test.sh [package_name]`

source /opt/ros/lyrical/setup.bash

if [ -z "$1" ]; then
    colcon test --ctest-args -L 'gtest|pytest'
else
    colcon test --packages-select "$1" --ctest-args -L 'gtest|pytest'
fi

if [ $? -ne 0 ]; then
    echo "Tests failed."
    exit 1
fi

echo ""
echo "========== Test Results =========="
colcon test-result --all

if [ $? -eq 0 ]; then
    echo "Test results retrieved successfully."
else
    echo "Failed to retrieve test results."
    exit 1
fi