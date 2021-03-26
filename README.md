Getting some ROS-based algorithms to run in DonkeyGym.

Paul and Michael: Please open Dockerfile and edit as needed. Some tasks are commented in there.

VS code has extension for docker and works well for me.

## Test

Build (image named donkey): 

`docker build . -t donkey --rm`

Run intereactive mode (container named dk):

`docker run --name dk -it donkey` or `docker exec --name dk -it donkey`

Remove all dangling images:

`docker image prune`
