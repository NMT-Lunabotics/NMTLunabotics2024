#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# List the USB devices we need.

d435() {
    if lsusb | grep -q '8086:0b07 Intel Corp. USB3.1 Hub'; then
        echo $'\x1b[32mOK.\x1b[0m'
    else
        echo $'\x1b[31mMissing!\x1b[0m'
    fi
}

d455() {
    number=$(lsusb | grep '8086:0b5c Intel Corp. USB3.1 Hub' | wc -l)
    if [ "$number" == 2 ]; then
        echo $'\x1b[32mOK.\x1b[0m'
    elif [ "$number" == 1 ]; then
        echo $'\x1b[31m1 of 2\x1b[0m'
    else
        echo $'\x1b[31mMissing!\x1b[0m'
    fi
}

t265() {
    if lsusb | grep -q '03e7:2150 Intel Myriad VPU'; then
        echo $'\x1b[32mMyriad mode.\x1b[0m'
    elif lsusb | grep -q '8087:0b37 Intel Corp.'; then
        echo $'\x1b[32mIntel mode.\x1b[0m'
    else
        echo $'\x1b[31mMissing!\x1b[0m'
    fi
}

usbcam() {
    if lsusb | grep -q '0c45:6366 Microdia Bluetooth Radio'; then
        echo $'\x1b[32mOK.\x1b[0m'
    else
        echo $'\x1b[31mMissing!\x1b[0m'
    fi
}

test_feature() {
    length=$(echo "$1" | wc -c)
    spaces=$((30 - length))

    echo -n "$1"
    for i in $(seq 1 $spaces); do
        echo -n ' '
    done

    "$2"
}

test_feature 'D435 camera...' d435
test_feature 'D455 cameras...' d455
test_feature 'T265 camera...' t265
test_feature 'USB camera...' usbcam
