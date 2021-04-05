#!/bin/bash
docker build . -t donkey --rm
docker rm dk
docker run --name dk -it donkey --entrypoint /bin/bash -c "/rlaunch.bash"