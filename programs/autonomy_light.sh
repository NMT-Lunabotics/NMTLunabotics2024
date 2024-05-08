#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

blink=1

while true; do
    if ps -e | grep -q move_base; then
        if [ $blink = 1 ]; then
            rostopic pub /leds/error std_msgs/Bool true --once &
        else
            rostopic pub /leds/error std_msgs/Bool false --once &
        fi
    else
        rostopic pub /leds/error std_msgs/Bool false --once &
    fi

    sleep 0.5
    blink=$((1 - blink))
done
