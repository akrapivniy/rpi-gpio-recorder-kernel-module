#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

DEVICE_NAME="rpigpio"
DRIVER_NAME="rpigpio-rec"

if [ $# -ne 1 ]; then
    echo "Usage: $0 <number_of_devices>"
    exit 1
fi

NUMBER_OF_DEVICES=$1

MAJOR_DEVICE_NUMBER=$(awk "/$DRIVER_NAME/ {print \$1}" /proc/devices)

if [ -z "$MAJOR_DEVICE_NUMBER" ]; then
    echo "No such driver found in /proc/devices"
    exit 1
fi

for (( i=0; i<${NUMBER_OF_DEVICES}; i++ ))
do
    DEVICE_PATH="/dev/${DEVICE_NAME}${i}"

    if [ -e "$DEVICE_PATH" ]; then
        unlink "$DEVICE_PATH"
    fi

    mknod "$DEVICE_PATH" c "$MAJOR_DEVICE_NUMBER" "$i"
    echo "Created $DEVICE_PATH"
done