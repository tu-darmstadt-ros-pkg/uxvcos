#!/bin/bash

cd $UXVCOS_ROOT/airplane/bin
echo "Running airplane-gnulinux with arguments $AIRPLANE_OPTIONS..."

sudo setcap cap_net_raw+ep ./airplane-gnulinux
roslaunch --wait airplane airplane.launch

