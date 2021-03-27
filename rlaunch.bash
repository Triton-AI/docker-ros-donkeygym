#!/bin/bash
echo "Starting config"

cd /catkin_ws
source devel/setup.bash
roslaunch donkey_gym_wrapper launch.launch

echo "Ending config"

/bin/bash "$@"