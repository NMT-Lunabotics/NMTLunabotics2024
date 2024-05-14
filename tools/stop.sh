#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Sends a rapid fire of CAN messages that stop the motors.

# This needs to run as sudo.
if [ "$(whoami)" != root ]; then
    exec sudo "$0"
fi

# Shut down autonomy that can't be cancelled.
killall digging_autonomy dumping_autonomy move_base

# Shut down move_base. This runs in the background to avoid blocking
# while we wait for ROS to load.
/var/ros/nixWrappers/rostopic pub move_base/cancel actionlib_msgs/GoalID -- '{}' &

while true; do
    # Stop the motors.
    cansend can0 001#00800080

    # Stop the actuators.
    cansend can0 003#80800000

    sleep 0.01
done
