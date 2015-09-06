#!/bin/sh

./bitalino/test &
sudo ./build/pd_motor_control 0.03 0.0001 10
