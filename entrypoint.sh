#!/bin/bash

# Set the XDG_RUNTIME_DIR environment variable
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Use software rendering as a fallback
export QT_XCB_GL_INTEGRATION=none

source /opt/ros/noetic/setup.bash
catkin build
source ./devel/setup.bash

# Start an interactive shell
exec "$@"