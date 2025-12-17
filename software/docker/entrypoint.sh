#!/bin/bash

http-server -p 80 /firmware/dist &

source /firmware/ros/setup.bash
ros2 launch bringup launch.xml