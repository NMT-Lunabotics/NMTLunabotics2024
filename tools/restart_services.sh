#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Restart all ROS services.

set -x

sudo systemctl stop 'ros-*'
sudo systemctl stop rosMaster
sudo systemctl start rosMaster
sudo systemctl start --all 'ros-*'
