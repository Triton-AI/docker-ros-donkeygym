#!/bin/bash
docker rm dk
docker build . -t donkey --rm
docker run -it --name dk --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --network="host" --entrypoint="/rlaunch.bash" donkey 