#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# List the USB devices we need.

d435() {
    lsusb | grep -q '8086:0b07 Intel Corp. USB3.1 Hub'
}

d455() {
    number=$(lsusb | grep '8086:0b5c Intel Corp. USB3.1 Hub' | wc -l)
    [ "$number" == 2 ]
}

t265() {
    lsusb | grep -q '03e7:2150 Intel Myriad VPU'
}

usbcam() {
    lsusb | grep -q '0c45:6366 Microdia Bluetooth Radio'
}

test_feature() {
    length=$(echo "$1" | wc -c)
    spaces=$((30 - length))

    echo -n "$1"
    for i in $(seq 1 $spaces); do
        echo -n ' '
    done

    if "$2"; then
        echo $'\x1b[32mOK.\x1b[0m'
    else
        echo $'\x1b[31mMissing!\x1b[0m'
    fi
}

test_feature 'D435 camera...' d435
test_feature 'D455 cameras...' d455
test_feature 'T265 camera...' t265
test_feature 'USB camera...' usbcam
