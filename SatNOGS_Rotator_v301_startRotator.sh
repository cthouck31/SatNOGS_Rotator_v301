#!/bin/bash

# Settings.
VERBOSITY=-vvvvvv
RETRIES=1
TIMEOUT=750

# IP address for GPredict.
IP_ADDR=127.0.0.1
PORT=4533

# Serial rate of Arduino.
SERIAL_DEV=/dev/ttyACM0
SERIAL_RATE=115200

# Easycomm II.
MODEL=202

# Command.
rotctld -m ${MODEL} -r ${SERIAL_DEV} -s ${SERIAL_RATE} -T ${IP_ADDR} -t ${PORT} -C timeout=${TIMEOUT} -C retry=${RETRIES} ${VERBOSITY}
