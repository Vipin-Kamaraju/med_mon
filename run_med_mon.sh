#!/bin/bash

xhost +local: # Allows the container to connect to the X server

docker run -it --rm \
    --name med_mon_app \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/dri:/dev/dri \
    -v /dev/video0:/dev/video0 \
    -v {ABSOLUTE_PATH_TO_MEDMON_DIR}:/med_mon \
    -e DISPLAY=$DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    --net=host \
    med_mon_image