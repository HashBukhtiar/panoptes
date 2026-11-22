#!/bin/bash

# Usage: `source scripts/setup.sh [ros_distro]`

ROS_DISTRO=${1:-lyrical}

source "/opt/ros/$ROS_DISTRO/setup.bash" || {
    echo "Failed to source ROS 2 $ROS_DISTRO."
    return 1
}

source install/setup.bash || {
    echo "Failed to source workspace."
    return 1
}

echo "ROS 2 ($ROS_DISTRO) and workspace sourced."