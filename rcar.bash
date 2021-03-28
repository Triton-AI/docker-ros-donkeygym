#!/bin/bash
echo "Starting config"

cd /catkin_ws
source devel/setup.bash
roslaunch ocvfiltercar rcar.launch

echo "Ending config"

/bin/bash "$@"