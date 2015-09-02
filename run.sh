#!/bin/sh

./bitalino_record &
sudo ./pd_motor_control 0.03 0.0001 10
