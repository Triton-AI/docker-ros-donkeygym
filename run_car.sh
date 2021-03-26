#!/bin/bash
source "catkin_ws/devel/setup.bash"
roslaunch donkey_gym_wrapper launch.launch & roslaunch ocvfiltercar rcar.launch