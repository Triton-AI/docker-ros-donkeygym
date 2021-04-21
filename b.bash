#!/bin/bash
sleep 0.5
docker image prune -f
docker exec -it dk bash ./rcar.bash
