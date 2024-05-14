#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Sends a move_base stop.

# Shut down move_base. This runs in the background to avoid blocking
# while we wait for ROS to load.
/var/ros/nixWrappers/rostopic pub move_base/cancel actionlib_msgs/GoalID -- '{}'
