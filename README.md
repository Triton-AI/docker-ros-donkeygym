Getting some ROS-based algorithms to run in DonkeyGym.

Paul and Michael: Please open Dockerfile and edit as needed. Some tasks are commented in there.

VS code has extension for docker and works well for me.

## Test

Build (image named donkey): 

`docker build . -t donkey --rm`

Run intereactive mode (container named dk):

`docker run --name dk -it donkey bash` or `docker exec -it dk bash`

Remove all dangling images:

`docker image prune`

Start a container:

`docker start -i dk`

Publish a ros topic:

`rostopic pub /drive ackermann_msgs/AckermannDriveStamped '{drive: {speed: 1}}'`


Note:
Run Docker after building donkey:
- xhost +local:root
- docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --network="host" donkey

To open a new terminal in container:
- docker ps
- docker exec -it [name] bash