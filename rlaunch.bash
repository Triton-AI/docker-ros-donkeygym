#!/bin/bash
echo "Starting config"

cd /catkin_ws
source devel/setup.bash
roslaunch zed_wrapper zed.launch

echo "Ending config"

/bin/bash "$@"